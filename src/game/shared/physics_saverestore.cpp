//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//=============================================================================//

#include "cbase.h"

#include "utlpriorityqueue.h"
#include "utlmap.h"
#include "isaverestore.h"
#include "physics.h"
#include "physics_saverestore.h"
#include "saverestoretypes.h"
#include "gamestringpool.h"
#include "datacache/imdlcache.h"

#if !defined( CLIENT_DLL )
#include "entitylist.h"
#endif

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// Log every pointer the repair table puts back together (the summary prints either
// way). Replicated because this file is compiled into both the client and server.
static ConVar phys_saverestore_repair_log( "phys_saverestore_repair_log", "0", FCVAR_REPLICATED, "Log each x64 physics pointer read repaired by the physics save/restore repair table." );

//-----------------------------------------------------------------------------

// Version 6 made the block header raw and version 7 added the repair table; older
// blocks are rejected (see ReadRestoreHeaders()).
static short PHYS_SAVE_RESTORE_VERSION = 7;

// Upper bound on the repair table, so a damaged save cannot loop over a nonsense
// count; m_RepairLookup is int-indexed and can represent all of them.
#define MAX_PHYS_POINTER_REPAIRS	( 1 << 18 )

// Written and read raw at pointer width: a data description would truncate the
// pointer on x64 or restore it as an entity. See ReadRestoreHeaders().
struct PhysBlockHeader_t
{
	int				nSaved;
	IPhysicsObject	*pWorldObject;

	inline void Clear()
	{
		nSaved = 0;
		pWorldObject = 0;
	}
};

// A (low32, high32) pair for a pointer vphysics saved through its 32 bit path.
// Entries whose low half is shared by two live pointers are never written out.
struct PhysPointerRepair_t
{
	unsigned int	m_nLow;
	unsigned int	m_nHigh;
	bool			m_bAmbiguous;
};

#if defined(_STATIC_LINKED) && defined(CLIENT_DLL)
const char *g_ppszPhysTypeNames[PIID_NUM_TYPES] =
{
	"Unknown",
	"IPhysicsObject",
	"IPhysicsFluidController",
	"IPhysicsSpring",
	"IPhysicsConstraintGroup",
	"IPhysicsConstraint",
	"IPhysicsShadowController",
	"IPhysicsPlayerController",
	"IPhysicsMotionController",
	"IPhysicsVehicleController",
};
#endif

//-----------------------------------------------------------------------------

struct BBox_t
{
	Vector mins, maxs;
};

struct Sphere_t
{
	float radius;
};


struct PhysObjectHeader_t
{
	PhysObjectHeader_t()
	{
		memset( this, 0, sizeof(*this) );
	}

	PhysInterfaceId_t 	type;
	EHANDLE				hEntity;
	string_t			fieldName;
	int 				nObjects;
	string_t			modelName;
	BBox_t				bbox;
	Sphere_t			sphere;
	int					iCollide;
	
	DECLARE_SIMPLE_DATADESC();
};

BEGIN_SIMPLE_DATADESC( PhysObjectHeader_t )
  	DEFINE_FIELD( type,			FIELD_INTEGER ),
  	DEFINE_FIELD( hEntity,		FIELD_EHANDLE ),
  	DEFINE_FIELD( fieldName,	FIELD_STRING ),
  	DEFINE_FIELD( nObjects,		FIELD_INTEGER ),
  	DEFINE_FIELD( modelName,	FIELD_STRING ),

	// Silence, Classcheck!
	// DEFINE_FIELD( bbox, BBox_t ),
	// DEFINE_FIELD( sphere, Sphere_t ),

  	DEFINE_FIELD( bbox.mins,	FIELD_VECTOR ),
  	DEFINE_FIELD( bbox.maxs,	FIELD_VECTOR ),
  	DEFINE_FIELD( sphere.radius, FIELD_FLOAT ),
  	DEFINE_FIELD( iCollide,		FIELD_INTEGER ),
END_DATADESC()

//-----------------------------------------------------------------------------
// Purpose:	The central manager of physics save/load
//

class CPhysSaveRestoreBlockHandler : public CDefSaveRestoreBlockHandler, 
									 public IPhysSaveRestoreManager
#if !defined( CLIENT_DLL )
									 , public IEntityListener
#endif
{
	struct QueuedItem_t;
public:
	CPhysSaveRestoreBlockHandler()
	{
		m_QueuedSaves.SetLessFunc( SaveQueueFunc );
		SetDefLessFunc( m_QueuedRestores );
		SetDefLessFunc( m_PhysObjectModels );
		SetDefLessFunc( m_PhysObjectCustomModels );
		SetDefLessFunc( m_PhysCollideBBoxModels );
		SetDefLessFunc( m_RepairLookup );

		m_nActivePhysObjectRestores = 0;
		m_nRepairedReads = 0;
	}

	const char *GetBlockName()
	{
		return "Physics";
	}

	// vphysics writes pointers through the 32 bit path, losing their upper half on x64;
	// Save() records them and this puts that half back. Returns true when repaired.
	bool TryRepairIntRead( int *pValue, int nElems )
	{
#if defined(PLATFORM_64BITS)
		// Only a single element read, into an aligned pointer sized slot, made
		// while vphysics itself is restoring, can be one of its pointers.
		if ( nElems != 1 || m_nActivePhysObjectRestores <= 0 )
			return false;

		if ( ( (uintp)pValue & ( sizeof( void * ) - 1 ) ) != 0 )
			return false;

		if ( m_RepairLookup.Count() == 0 )
			return false;

		unsigned int nLow = (unsigned int)(uintp)*pValue;

		int iLookup = m_RepairLookup.Find( nLow );
		if ( iLookup == m_RepairLookup.InvalidIndex() )
			return false;

		int iEntry = m_RepairLookup[iLookup];
		if ( iEntry < 0 )
			return false;	// ambiguous low half, leave it alone

		// pValue points at the low 32 bits of the slot vphysics read into.
		unsigned int nHigh = m_RepairEntries[iEntry].m_nHigh;
		*(uintp *)pValue = ( (uintp)nHigh << 32 ) | (uintp)nLow;
		++m_nRepairedReads;

		if ( phys_saverestore_repair_log.GetBool() )
		{
			Msg( "Physics save/restore: repaired a pointer read, low 0x%08X high 0x%08X\n", nLow, nHigh );
		}

		return true;
#else
		UNREFERENCED_PARAMETER( pValue );
		UNREFERENCED_PARAMETER( nElems );
		return false;
#endif
	}

	//---------------------------------

	virtual void PreSave( CSaveRestoreData * ) 
	{
		m_blockHeader.Clear();
		ClearPhysPointerRepairs();
	}
	
	//---------------------------------

	virtual void Save( ISave *pSave ) 
	{
		m_blockHeader.pWorldObject = g_PhysWorldObject;
		m_blockHeader.nSaved = m_QueuedSaves.Count();

		// The world object belongs in the repair table too; write the table before
		// any object data.
		NotePhysPointerForRepair( g_PhysWorldObject );
		WritePhysPointerRepairTable( pSave );

		while ( m_QueuedSaves.Count() )
		{
			const QueuedItem_t &item = m_QueuedSaves.ElementAtHead();
			
			CBaseEntity *pOwner = item.header.hEntity.Get();
			
			if ( pOwner )
			{
				pSave->WriteAll( &item.header );
				pSave->StartBlock(); //  Need block here in case entity is NULL on load
				if ( item.header.nObjects )
				{
					for ( int i = 0; i < item.header.nObjects; i++ )
					{
						// Starting a block here allows the implementation of any individual physics
						// class save/load to change non-trivially while retaining the overall
						// integrity of the savefile.
						pSave->StartBlock();
						SavePhysicsObject( pSave, pOwner, item.ppPhysObj[i], item.header.type );
						pSave->EndBlock();
					}
				}
				// else, it will simply be recreated on restore
				pSave->EndBlock();
			}
			m_QueuedSaves.RemoveAtHead();
		}
	}
	
	//---------------------------------

	virtual void WriteSaveHeaders( ISave *pSave )
	{
		pSave->WriteShort( &PHYS_SAVE_RESTORE_VERSION );

		// Written raw: WriteAll() would go through a data description and
		// truncate the pointer. See PhysBlockHeader_t.
		pSave->WriteData( (const char *)&m_blockHeader.nSaved, sizeof( m_blockHeader.nSaved ) );
		pSave->WriteData( (const char *)&m_blockHeader.pWorldObject, sizeof( m_blockHeader.pWorldObject ) );
	}
	
	//---------------------------------

	virtual void PostSave() 
	{
		m_QueuedSaves.Purge();
	}
	
	//---------------------------------

	virtual void PreRestore() 
	{
#if !defined( CLIENT_DLL )
		gEntList.AddListenerEntity( this );
#endif

		// UNDONE: This never runs!!!!
		if ( physenv )
		{
			physprerestoreparams_t params;
			params.recreatedObjectCount = 0;
			physenv->PreRestore( params );
		}
	}
	
	//---------------------------------

	virtual void ReadRestoreHeaders( IRestore *pRestore )
	{
		// A version mismatch rejects the load: a skipped block would leave physics
		// fields the entity pass has already cleared.
		short version = pRestore->ReadShort();
		if ( version != PHYS_SAVE_RESTORE_VERSION )
		{
			Error( "Physics save data is version %d, but this build writes and reads version %d. "
				   "The save was written by an incompatible build and cannot be loaded safely.\n",
				   (int)version, (int)PHYS_SAVE_RESTORE_VERSION );
		}

		// Raw, pointer width header - see WriteSaveHeaders().
		pRestore->ReadData( (char *)&m_blockHeader.nSaved, sizeof( m_blockHeader.nSaved ), sizeof( m_blockHeader.nSaved ) );
		pRestore->ReadData( (char *)&m_blockHeader.pWorldObject, sizeof( m_blockHeader.pWorldObject ), sizeof( m_blockHeader.pWorldObject ) );
	}

	//---------------------------------
	
	virtual void Restore( IRestore *pRestore, bool ) 
	{
		// Read the repair table first: vphysics needs it in place before it
		// reads any of the pointers it saved.
		ReadPhysPointerRepairTable( pRestore );

		if ( physenv )
		{
			physprerestoreparams_t params;
			params.recreatedObjectCount = 1;
			params.recreatedObjectList[0].pNewObject = g_PhysWorldObject;
			params.recreatedObjectList[0].pOldObject = m_blockHeader.pWorldObject;
			physenv->PreRestore( params );
		}

		PhysObjectHeader_t header;

		while ( m_blockHeader.nSaved-- )
		{
			pRestore->ReadAll( &header );
			pRestore->StartBlock();
			
			if ( header.hEntity != NULL )
			{
				RestoreBlock( pRestore, header );
			}

			pRestore->EndBlock();
		}

		Msg( "Physics save/restore: %d pointer(s) in the repair table, %d int read(s) repaired\n", m_RepairLookup.Count(), m_nRepairedReads );
	}
	
	//---------------------------------
	
	void RestoreBlock( IRestore *pRestore, const PhysObjectHeader_t &header ) 
	{
		CBaseEntity *  pOwner  = header.hEntity.Get();
		unsigned short iQueued = m_QueuedRestores.Find( pOwner );
		
		if ( iQueued != m_QueuedRestores.InvalidIndex() )
		{
			MDLCACHE_CRITICAL_SECTION();
			if ( pOwner->ShouldSavePhysics() && header.nObjects > 0 )
			{
				QueuedItem_t *pItem = m_QueuedRestores[iQueued]->FindItem( header.fieldName );
				
				if ( pItem )
				{
					int nObjects = MIN( header.nObjects, pItem->header.nObjects );
					if ( pItem->header.type == PIID_IPHYSICSOBJECT && nObjects == 1 )
					{
						RestorePhysicsObjectAndModel( pRestore, header, pItem, nObjects );
					}
					else
					{
						void **ppPhysObj = pItem->ppPhysObj;
						
						for ( int i = 0; i < nObjects; i++ )
						{
							pRestore->StartBlock();
							RestorePhysicsObject( pRestore, header, ppPhysObj + i );
							pRestore->EndBlock();
							if ( header.type == PIID_IPHYSICSMOTIONCONTROLLER )
							{
								void *pObj = ppPhysObj[i];
								IPhysicsMotionController *pController = (IPhysicsMotionController *)pObj;
								if ( pController )
								{
									// If the entity is the motion callback handler, then automatically set it
									// NOTE: This is usually the case
									IMotionEvent *pEvent = dynamic_cast<IMotionEvent *>(pOwner);
									if ( pEvent )
									{
										pController->SetEventHandler( pEvent );
									}
								}
							}
						}
					}
				}
			}
			else
				pOwner->CreateVPhysics();
		}
	}
	
	
	//---------------------------------

	void RestorePhysicsObjectAndModel( IRestore *pRestore, const PhysObjectHeader_t &header, CPhysSaveRestoreBlockHandler::QueuedItem_t *pItem, int nObjects )
	{
		if ( nObjects == 1 )
		{
			pRestore->StartBlock();
			
			CPhysCollide *pPhysCollide   = NULL;
			int 		  modelIndex 	 = -1;
			bool 		  fCustomCollide = false;
			
			if ( header.modelName != NULL_STRING )
			{
				CBaseEntity *pGlobalEntity = header.hEntity;
#if !defined( CLIENT_DLL )
				if ( NULL_STRING != pGlobalEntity->m_iGlobalname )
				{
					modelIndex = pGlobalEntity->GetModelIndex();
				}
				else
#endif
				{
					modelIndex = modelinfo->GetModelIndex( STRING( header.modelName ) );
					pGlobalEntity = NULL;
				}

				if ( modelIndex != -1 )
				{
					vcollide_t *pCollide = modelinfo->GetVCollide( modelIndex );
					if ( pCollide )
					{
						if ( pCollide->solidCount > 0 && pCollide->solids && header.iCollide < pCollide->solidCount )
							pPhysCollide = pCollide->solids[header.iCollide];
					}
				}
			}
			else if ( header.bbox.mins != vec3_origin || header.bbox.maxs != vec3_origin )
			{
				pPhysCollide = PhysCreateBbox( header.bbox.mins, header.bbox.maxs );
				fCustomCollide = true;
			}
			else if ( header.sphere.radius != 0 )
			{
				// HACKHACK: Handle spheres here!!!
				if ( !(*pItem->ppPhysObj) )
				{
					RestorePhysicsObject( pRestore, header, pItem->ppPhysObj, NULL );
				}
				return;
			}
			
			if ( pPhysCollide )
			{
				if ( !(*pItem->ppPhysObj) )
				{
					RestorePhysicsObject( pRestore, header, pItem->ppPhysObj, pPhysCollide );
					if ( (*pItem->ppPhysObj) )
					{
						IPhysicsObject *pObject = (IPhysicsObject *)(*pItem->ppPhysObj);
						if ( !fCustomCollide )
						{
							AssociateModel( pObject, modelIndex );
						}
						else
						{
							AssociateModel( pObject, pPhysCollide );
						}
					}
					else
						DevMsg( "Failed to restore physics object\n" );
				}
				else
					DevMsg( "Physics object pointer unexpectedly non-null before restore. Should be creating physics object in CreatePhysics()?\n" );
			}
			else
				DevMsg( "Failed to reestablish collision model for object\n" );
				
			pRestore->EndBlock();
		}
		else
			DevMsg( "Don't know how to reconsitite models for physobj array \n" );
	}
	
	//---------------------------------
	
	virtual void PostRestore() 
	{
		if ( physenv )
			physenv->PostRestore();

		unsigned short i = m_QueuedRestores.FirstInorder();
		while ( i != m_QueuedRestores.InvalidIndex() )
		{
			delete m_QueuedRestores[i];
			i = m_QueuedRestores.NextInorder( i );
		}
		
		m_QueuedRestores.RemoveAll();
#if !defined( CLIENT_DLL )
		gEntList.RemoveListenerEntity( this );
#endif
		ClearPhysPointerRepairs();
	}
	
	//---------------------------------
	
	void QueueSave( CBaseEntity *pOwner, typedescription_t *pTypeDesc, void **ppPhysObj, PhysInterfaceId_t type )
	{
		if ( !pOwner )
			return;

		bool fOnlyNotingExistence = !pOwner->ShouldSavePhysics();
		
		QueuedItem_t item;
		
		item.ppPhysObj		= ppPhysObj;
		item.header.hEntity = pOwner;
		item.header.type	= type;
		item.header.nObjects = ( !fOnlyNotingExistence ) ? pTypeDesc->fieldSize : 0;
		item.header.fieldName = AllocPooledString( pTypeDesc->fieldName ); 	
																	// A pooled string is used here because there is no way
																	// right now to save a non-string_t string and have it 
																	// compressed in the save symbol tables. Furthermore,
																	// the field name would normally be in the string
																	// pool anyway. (toml 12-10-02)
		item.header.modelName = NULL_STRING;
		memset( &item.header.bbox, 0, sizeof( item.header.bbox ) );
		item.header.sphere.radius = 0;

		if ( !fOnlyNotingExistence )
		{
			// Every pointer handed to vphysics can come back through a 32 bit
			// read, arrays included.
			for ( int i = 0; i < pTypeDesc->fieldSize; ++i )
			{
				NotePhysPointerForRepair( ppPhysObj[i] );
			}
		}
		
		if ( !fOnlyNotingExistence && type == PIID_IPHYSICSOBJECT )
		{
			// Don't doing the box thing for things like wheels on cars
			IPhysicsObject *pPhysObj = (IPhysicsObject *)(*ppPhysObj);

			if ( pPhysObj )
			{
				item.header.modelName = GetModelName( pPhysObj );
				item.header.iCollide = physcollision->CollideIndex( pPhysObj->GetCollide() );
				if ( item.header.modelName == NULL_STRING )
				{
					BBox_t *pBBox = GetBBox( pPhysObj );
					if ( pBBox != NULL )
					{
						item.header.bbox = *pBBox;
					}
					else 
					{
						if ( pPhysObj && pPhysObj->GetSphereRadius() != 0 )
						{
							item.header.sphere.radius = pPhysObj->GetSphereRadius();
						}
						else
						{
							DevMsg( "Don't know how to save model for physics object (class \"%s\")\n", pOwner->GetClassname() );
						}
					}
				}
			}
		}

		m_QueuedSaves.Insert( item );
	}

	//---------------------------------
	
	void QueueRestore( CBaseEntity *pOwner, typedescription_t *pTypeDesc, void **ppPhysObj, PhysInterfaceId_t type )
	{
		CEntityRestoreSet *pEntitySet = NULL;
		unsigned short 	   iEntitySet = m_QueuedRestores.Find( pOwner );
		
		if ( iEntitySet != m_QueuedRestores.InvalidIndex() )
		{
			pEntitySet = m_QueuedRestores[iEntitySet];
		}
		else
		{
			pEntitySet = new CEntityRestoreSet;
			m_QueuedRestores.Insert( pOwner, pEntitySet );
		}
		
		pEntitySet->Add( pOwner, pTypeDesc, ppPhysObj, type );

		memset( ppPhysObj, 0, pTypeDesc->fieldSize * sizeof( void * ) );
	}

	//---------------------------------
	
	void SavePhysicsObject( ISave *pSave, CBaseEntity *pOwner, void *pObject, PhysInterfaceId_t type )
	{
		if ( physenv )
		{
			if ( !pObject )
				return;
			physsaveparams_t params = { pSave, pObject, type };
			physenv->Save( params );
		}
	}
	
	//---------------------------------
	
	void RestorePhysicsObject( IRestore *pRestore, const PhysObjectHeader_t &header, void **ppObject, const CPhysCollide *pCollide = NULL )
	{
		if ( physenv )
		{
			physrestoreparams_t params = { pRestore, ppObject, header.type, header.hEntity.Get(), STRING(header.modelName), pCollide, physenv, physgametrace };

			// vphysics reads the pointers it saved through pRestore while this
			// call is on the stack; only those reads may be repaired.
			++m_nActivePhysObjectRestores;
			physenv->Restore( params );
			--m_nActivePhysObjectRestores;
		}
	}
#if !defined( CLIENT_DLL )	
	//-----------------------------------------------------
	// IEntityListener methods
	// This object is only a listener during restore	
	virtual void OnEntityCreated( CBaseEntity *pEntity )
	{
	}

	//---------------------------------
	
	virtual void OnEntityDeleted( CBaseEntity *pEntity )
	{
		unsigned short iEntitySet = m_QueuedRestores.Find( pEntity );
		
		if ( iEntitySet != m_QueuedRestores.InvalidIndex() )
		{
			delete m_QueuedRestores[iEntitySet];
			m_QueuedRestores.RemoveAt( iEntitySet );
		}
	}
#endif

	//-----------------------------------------------------
	// IPhysSaveRestoreManager methods
	
	virtual void NoteBBox( const Vector &mins, const Vector &maxs, CPhysCollide *pCollide )
	{
		if ( pCollide && m_PhysCollideBBoxModels.Find( pCollide ) == m_PhysCollideBBoxModels.InvalidIndex() )
		{
			BBox_t box;
			box.mins = mins;
			box.maxs = maxs;
			m_PhysCollideBBoxModels.Insert( pCollide, box );
		}
	}

	//---------------------------------
	
	virtual void AssociateModel( IPhysicsObject *pObject, int modelIndex )
	{
		Assert( m_PhysObjectModels.Find( pObject ) == m_PhysObjectModels.InvalidIndex() );
		m_PhysObjectModels.Insert( pObject, modelIndex );
	}

	//---------------------------------
	
	virtual void AssociateModel( IPhysicsObject *pObject, const CPhysCollide *pModel )
	{
		Assert( m_PhysObjectCustomModels.Find( pObject ) == m_PhysObjectCustomModels.InvalidIndex() );
		m_PhysObjectCustomModels.Insert( pObject, pModel );
	}

	//---------------------------------
	
	virtual void ForgetModel( IPhysicsObject *pObject )
	{
		if ( !m_PhysObjectModels.Remove( pObject ) )
			m_PhysObjectCustomModels.Remove( pObject );
	}

	//---------------------------------

	virtual void ForgetAllModels()
	{
		m_PhysObjectModels.RemoveAll();
		m_PhysObjectCustomModels.RemoveAll();
		m_PhysCollideBBoxModels.RemoveAll();
	}

	//---------------------------------
	
	string_t GetModelName( IPhysicsObject *pObject )
	{
		int i = m_PhysObjectModels.Find( pObject );
		if ( i == m_PhysObjectModels.InvalidIndex() )
			return NULL_STRING;
		return AllocPooledString( modelinfo->GetModelName( modelinfo->GetModel( m_PhysObjectModels[i] ) ) );
	}
	
	//---------------------------------
	
	BBox_t * GetBBox( IPhysicsObject *pObject )
	{
		int i = m_PhysObjectCustomModels.Find( pObject );
		if ( i == m_PhysObjectCustomModels.InvalidIndex() )
			return NULL;
		i = m_PhysCollideBBoxModels.Find( m_PhysObjectCustomModels[i] );
		if ( i == m_PhysCollideBBoxModels.InvalidIndex() )
			return NULL;
		return &(m_PhysCollideBBoxModels[i]);
	}

	//---------------------------------
	
private:
	struct QueuedItem_t
	{
		PhysObjectHeader_t	  header;
	 	void **				  ppPhysObj;
	};
	
	class CEntityRestoreSet : public CUtlVector<QueuedItem_t>
	{
	public:
		int Add( CBaseEntity *pOwner, typedescription_t *pTypeDesc, void **ppPhysObj, PhysInterfaceId_t type )
		{
			int i = AddToTail();
			
			Assert( ppPhysObj );
			Assert( *ppPhysObj == NULL ); // expected field to have been cleared
			Assert( pOwner );

			QueuedItem_t &item = Element( i );

			item.ppPhysObj			= ppPhysObj;
			item.header.hEntity 	= pOwner;
			item.header.type		= type;
			item.header.nObjects 	= pTypeDesc->fieldSize;
			item.header.fieldName 	= AllocPooledString( pTypeDesc->fieldName ); 	// See comment in CPhysSaveRestoreBlockHandler::QueueSave()
			
			return i;
		}
		
		QueuedItem_t *FindItem( string_t itemFieldName )
		{
			// generally, the set is very small, usually one, so linear search is not too gruesome;
			for ( int i = 0; i < Count(); i++ )
			{
				string_t testName = Element(i).header.fieldName;
				Assert( ( testName == itemFieldName && strcmp( STRING( testName ), STRING( itemFieldName ) ) == 0 ) ||
						( testName != itemFieldName && strcmp( STRING( testName ), STRING( itemFieldName ) ) != 0 ) );
				
				if ( testName == itemFieldName )
					return &(Element(i));
			}
			return NULL;
		}
	};
	
	//---------------------------------
	
	static bool SaveQueueFunc( const QueuedItem_t &left, const QueuedItem_t &right )
	{
		if ( left.header.type == right.header.type )
			return ( left.header.hEntity->entindex() > right.header.hEntity->entindex() );

		return ( left.header.type > right.header.type );
	}
	
	//---------------------------------

	CUtlPriorityQueue<QueuedItem_t> 			m_QueuedSaves;
	CUtlMap<CBaseEntity *, CEntityRestoreSet *>	m_QueuedRestores;

	//---------------------------------
	
	CUtlMap<IPhysicsObject *, int>					m_PhysObjectModels;
	CUtlMap<IPhysicsObject *, const CPhysCollide *>	m_PhysObjectCustomModels;
	CUtlMap<const CPhysCollide *, BBox_t>			m_PhysCollideBBoxModels;

	//---------------------------------
	
	PhysBlockHeader_t							m_blockHeader;

	//---------------------------------
	// x64 pointer repair - see TryRepairIntRead()

	void ClearPhysPointerRepairs()
	{
		m_RepairEntries.RemoveAll();
		m_RepairLookup.RemoveAll();
		m_nActivePhysObjectRestores = 0;
		m_nRepairedReads = 0;
	}

	void NotePhysPointerForRepair( const void *pPointer )
	{
#if defined(PLATFORM_64BITS)
		if ( !pPointer )
			return;

		uintp nPointer = (uintp)pPointer;
		unsigned int nLow = (unsigned int)nPointer;
		unsigned int nHigh = (unsigned int)( nPointer >> 32 );

		int iLookup = m_RepairLookup.Find( nLow );
		if ( iLookup == m_RepairLookup.InvalidIndex() )
		{
			PhysPointerRepair_t entry;
			entry.m_nLow = nLow;
			entry.m_nHigh = nHigh;
			entry.m_bAmbiguous = false;
			m_RepairLookup.Insert( nLow, m_RepairEntries.AddToTail( entry ) );
			return;
		}

		int iEntry = m_RepairLookup[iLookup];
		if ( iEntry < 0 )
			return;	// already ambiguous

		if ( m_RepairEntries[iEntry].m_nHigh != nHigh )
		{
			// Two live pointers share a low half and disagree about the upper
			// half: repairing either of them would be a guess, so refuse both.
			m_RepairEntries[iEntry].m_bAmbiguous = true;
			m_RepairLookup[iLookup] = -1;
		}
#else
		UNREFERENCED_PARAMETER( pPointer );
#endif
	}

	void WritePhysPointerRepairTable( ISave *pSave )
	{
		int nAmbiguous = 0;
		for ( int i = 0; i < m_RepairEntries.Count(); ++i )
		{
			if ( m_RepairEntries[i].m_bAmbiguous )
				++nAmbiguous;
		}

		int nWritten = m_RepairEntries.Count() - nAmbiguous;
		pSave->WriteInt( &nWritten );

		for ( int i = 0; i < m_RepairEntries.Count(); ++i )
		{
			if ( m_RepairEntries[i].m_bAmbiguous )
				continue;

			int nLow = (int)m_RepairEntries[i].m_nLow;
			int nHigh = (int)m_RepairEntries[i].m_nHigh;
			pSave->WriteInt( &nLow );
			pSave->WriteInt( &nHigh );
		}

		DevMsg( "Physics save/restore: %d pointer(s) in the repair table (%d refused as ambiguous)\n", nWritten, nAmbiguous );
	}

	void ReadPhysPointerRepairTable( IRestore *pRestore )
	{
		int nEntries = 0;
		pRestore->ReadInt( &nEntries );

		// An impossible count means a corrupt block; reject the load.
		if ( nEntries < 0 || nEntries > MAX_PHYS_POINTER_REPAIRS )
		{
			Error( "Physics save data is corrupt: the repair table claims %d entries (maximum %d). "
				   "The save cannot be loaded safely.\n", nEntries, (int)MAX_PHYS_POINTER_REPAIRS );
		}

		for ( int i = 0; i < nEntries; ++i )
		{
			int nLow = 0;
			int nHigh = 0;
			pRestore->ReadInt( &nLow );
			pRestore->ReadInt( &nHigh );
			AddPhysPointerRepair( (unsigned int)nLow, (unsigned int)nHigh );
		}
	}

	void AddPhysPointerRepair( unsigned int nLow, unsigned int nHigh )
	{
		int iLookup = m_RepairLookup.Find( nLow );
		if ( iLookup == m_RepairLookup.InvalidIndex() )
		{
			PhysPointerRepair_t entry;
			entry.m_nLow = nLow;
			entry.m_nHigh = nHigh;
			entry.m_bAmbiguous = false;
			m_RepairLookup.Insert( nLow, m_RepairEntries.AddToTail( entry ) );
			return;
		}

		int iEntry = m_RepairLookup[iLookup];
		if ( iEntry < 0 )
			return;	// already ambiguous

		if ( m_RepairEntries[iEntry].m_nHigh != nHigh )
		{
			// A damaged table; refuse to repair this low half.
			m_RepairEntries[iEntry].m_bAmbiguous = true;
			m_RepairLookup[iLookup] = -1;
		}
	}

	CUtlVector<PhysPointerRepair_t>				m_RepairEntries;
	CUtlMap<unsigned int, int, int>				m_RepairLookup;
	int											m_nActivePhysObjectRestores;
	int											m_nRepairedReads;
};

//-----------------------------------------------------------------------------

CPhysSaveRestoreBlockHandler g_PhysSaveRestoreBlockHandler;

IPhysSaveRestoreManager *g_pPhysSaveRestoreManager = &g_PhysSaveRestoreBlockHandler;

//-------------------------------------

ISaveRestoreBlockHandler *GetPhysSaveRestoreBlockHandler()
{
	return &g_PhysSaveRestoreBlockHandler;
}

// Called by CRestore::ReadInt() for every int read from the stream; only reads
// vphysics makes while restoring its own objects are repaired.

bool PhysicsSaveRestoreRepairIntRead( int *pValue, int nElems )
{
	return g_PhysSaveRestoreBlockHandler.TryRepairIntRead( pValue, nElems );
}

static bool IsValidEntityPointer( void *ptr )
{
#if !defined( CLIENT_DLL )
	return gEntList.IsEntityPtr( ptr );
#else
	// Walk entities looking for pointer
	int c = ClientEntityList().GetHighestEntityIndex();
	for ( int i = 0; i <= c; i++ )
	{
		CBaseEntity *e = ClientEntityList().GetBaseEntity( i );
		if ( !e )
			continue;

		if ( e == ptr )
			return true;
	}
	return false;
#endif
}

//-----------------------------------------------------------------------------
// Purpose:	Classifies field and queues it up for physics save/restore.
//

class CPhysObjSaveRestoreOps : public CDefSaveRestoreOps
{
public:
	virtual void Save( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave )
	{
		CBaseEntity *pOwnerEntity = pSave->GetGameSaveRestoreInfo()->GetCurrentEntityContext();

		bool bFoundEntity = true;
		
		if ( IsValidEntityPointer(pOwnerEntity) == false )
		{
			bFoundEntity = false;

#if defined( CLIENT_DLL )
			pOwnerEntity = ClientEntityList().GetBaseEntityFromHandle( pOwnerEntity->GetRefEHandle() );

			if ( pOwnerEntity  )
			{
				bFoundEntity = true;
			}
#endif
		}

		AssertMsg( pOwnerEntity && bFoundEntity == true, "Physics save/load is only suitable for entities" );

		if ( m_type == PIID_UNKNOWN )
		{
			AssertMsg( 0, "Unknown physics save/load type");
			return;
		}
		g_PhysSaveRestoreBlockHandler.QueueSave( pOwnerEntity, fieldInfo.pTypeDesc, (void **)fieldInfo.pField, m_type );
	}
	
	virtual void Restore( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore )
	{
		CBaseEntity *pOwnerEntity = pRestore->GetGameSaveRestoreInfo()->GetCurrentEntityContext();

		bool bFoundEntity = true;
		
		if ( IsValidEntityPointer(pOwnerEntity) == false )
		{
			bFoundEntity = false;

#if defined( CLIENT_DLL )
			pOwnerEntity = ClientEntityList().GetBaseEntityFromHandle( pOwnerEntity->GetRefEHandle() );

			if ( pOwnerEntity  )
			{
				bFoundEntity = true;
			}
#endif
		}

		AssertMsg( pOwnerEntity && bFoundEntity == true, "Physics save/load is only suitable for entities" );

		if ( m_type == PIID_UNKNOWN )
		{
			AssertMsg( 0, "Unknown physics save/load type");
			return;
		}
		
		g_PhysSaveRestoreBlockHandler.QueueRestore( pOwnerEntity, fieldInfo.pTypeDesc, (void **)fieldInfo.pField, m_type );
	}
	
	virtual void MakeEmpty( const SaveRestoreFieldInfo_t &fieldInfo )
	{
		memset( fieldInfo.pField, 0, fieldInfo.pTypeDesc->fieldSize * sizeof( void * ) );
	}
	
	virtual bool IsEmpty( const SaveRestoreFieldInfo_t &fieldInfo )
	{
		void **ppPhysObj = (void **)fieldInfo.pField;
		int nObjects = fieldInfo.pTypeDesc->fieldSize;
		for ( int i = 0; i < nObjects; i++ )
		{
			if ( ppPhysObj[i] != NULL )
				return false;
		}
		return true;
	}
	
	PhysInterfaceId_t m_type;
};

//-----------------------------------------------------------------------------

CPhysObjSaveRestoreOps g_PhysObjSaveRestoreOps[PIID_NUM_TYPES];

//-------------------------------------

ISaveRestoreOps *GetPhysObjSaveRestoreOps( PhysInterfaceId_t type )
{
	static bool inited;
	if ( !inited )
	{
		inited = true;
		for ( int i = 0; i < PIID_NUM_TYPES; i++ )
		{
			g_PhysObjSaveRestoreOps[i].m_type = (PhysInterfaceId_t)i;
		}
	}
	return &g_PhysObjSaveRestoreOps[type];
}

//=============================================================================
