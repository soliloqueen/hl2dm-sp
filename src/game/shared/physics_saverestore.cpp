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
#include "tier1/fmtstr.h"

#if !defined( CLIENT_DLL )
#include "entitylist.h"
#endif

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// x64 vphysics saves pointers through 32 bit int reads/writes; the block handler recognizes those
// exact calls, writes dense ids, and zero extends them on restore.

// 5 stock, 6/7 earlier builds of this mod, 8 dense ids. 5-7 still load.
static short PHYS_SAVE_RESTORE_VERSION = 8;
#define PHYS_SAVE_RESTORE_OLDEST_VERSION	5

// Explains why the save can't be loaded and disconnects.
static void RejectPhysicsSave( const char *pszReason )
{
	Warning( "\n*** This save can't be loaded ***\n%s\n\n", pszReason );

#if defined( CLIENT_DLL )
	engine->ClientCmd_Unrestricted( "disconnect\nshowconsole\n" );
#else
	engine->ServerCommand( "disconnect\n" );
#endif
}

// Bound on the version 7 repair table, which is skipped.
#define MAX_PHYS_POINTER_REPAIRS	( 1 << 18 )

// Version 5 block header, read only. Names must match what version 5 wrote.
struct PhysBlockHeader_t
{
	int nSaved;
	int pWorldObject;

	DECLARE_SIMPLE_DATADESC();
};
BEGIN_SIMPLE_DATADESC( PhysBlockHeader_t )
	DEFINE_FIELD( nSaved,		FIELD_INTEGER ),
	DEFINE_FIELD( pWorldObject,	FIELD_INTEGER ),
END_DATADESC()

// vphysics's motion controller object list field.
#define VPHYS_OBJECT_LIST_FIELD	"m_objectList"

// Zeroes the upper half of the 8 byte slot vphysics read an id into.
static inline void ZeroPointerUpperHalf( int *pValue )
{
#if defined( PLATFORM_64BITS )
	pValue[1] = 0;
#else
	UNREFERENCED_PARAMETER( pValue );
#endif
}

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
		SetDefLessFunc( m_SaveIds );
		SetDefLessFunc( m_RestoredObjects );

		m_fDoLoad = false;
		m_bLegacyLayout = false;
		m_nLoadVersion = 0;
		m_nSavedCount = 0;
		m_nWorldObjectId = 0;
		ResetIdentityState();
	}

	const char *GetBlockName()
	{
		return "Physics";
	}

	//---------------------------------
	// Hooks called from CSave/CRestore

	// CSave::WriteInt: substitutes the id for a pointer vphysics is writing.
	bool RemapIntWrite( const int *pValue, int *pOut )
	{
		if ( m_nSaveDepth <= 0 )
			return false;

		if ( m_pIdentitySlot && pValue == (const int *)m_pIdentitySlot )
		{
			m_pIdentitySlot = NULL;
		}
		else if ( m_pPointerSlot && pValue == (const int *)m_pPointerSlot )
		{
			m_pPointerSlot = NULL;
		}
		else
		{
			return false;
		}

		*pOut = SaveIdFor( *(void * const *)pValue );
		++m_nIdentitiesWritten;
		return true;
	}

	// CRestore::ReadInt: zero extends a pointer id vphysics just read.
	void FixupIntRead( int *pValue, int nElems )
	{
		if ( m_nRestoreDepth <= 0 || nElems != 1 )
			return;

		if ( m_bExpectObjectIdentity )
		{
			// CPhysicsEnvironment::Restore reads the object's own id first.
			m_bExpectObjectIdentity = false;
			ZeroPointerUpperHalf( pValue );
			m_nLastObjectIdentity = (unsigned int)*pValue;
			++m_nIdentitiesRead;
		}
		else if ( m_pPointerSlot && pValue == (int *)m_pPointerSlot )
		{
			m_pPointerSlot = NULL;
			ZeroPointerUpperHalf( pValue );
			++m_nIdentitiesRead;
		}
	}

	// CSave::WriteBasicField, FIELD_CUSTOM.
	void SaveCustomField( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave )
	{
		typedescription_t *pTypeDesc = fieldInfo.pTypeDesc;
		ISaveRestoreOps *pOps = pTypeDesc->pSaveRestoreOps;

		if ( m_nSaveDepth <= 0 )
		{
			pOps->Save( fieldInfo, pSave );
			return;
		}

		void *pPrevSlot = m_pPointerSlot;

		if ( IsObjectListField( pTypeDesc ) )
		{
			// Count, then one id per element.
			m_pPointerSlot = NULL;
			const CUtlVector<void *> *pList = (const CUtlVector<void *> *)fieldInfo.pField;
			int nCount = pList->Count();
			pSave->WriteInt( &nCount );
			for ( int i = 0; i < nCount; ++i )
			{
				int nId = SaveIdFor( pList->Element( i ) );
				pSave->WriteInt( &nId );
			}
			++m_nListsHandled;
		}
		else if ( pTypeDesc->fieldSize > 1 )
		{
			// Pointer array: one element per call.
			typedescription_t single = *pTypeDesc;
			single.fieldSize = 1;
			for ( int i = 0; i < pTypeDesc->fieldSize; ++i )
			{
				void **ppSlot = (void **)fieldInfo.pField + i;
				SaveRestoreFieldInfo_t element = { ppSlot, fieldInfo.pOwner, &single };
				m_pPointerSlot = ppSlot;
				pOps->Save( element, pSave );
			}
			++m_nArraysHandled;
		}
		else
		{
			m_pPointerSlot = fieldInfo.pField;
			pOps->Save( fieldInfo, pSave );
		}

		m_pPointerSlot = pPrevSlot;
	}

	// CRestore::ReadBasicField, FIELD_CUSTOM. nBytes is the saved field size.
	void RestoreCustomField( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore, int nBytes )
	{
		typedescription_t *pTypeDesc = fieldInfo.pTypeDesc;
		ISaveRestoreOps *pOps = pTypeDesc->pSaveRestoreOps;

		if ( m_nRestoreDepth <= 0 )
		{
			pOps->Restore( fieldInfo, pRestore );
			return;
		}

		void *pPrevSlot = m_pPointerSlot;
		int nWords = nBytes / (int)sizeof( int );

		if ( IsObjectListField( pTypeDesc ) )
		{
			m_pPointerSlot = NULL;
			RestoreObjectList( (CUtlVector<void *> *)fieldInfo.pField, pRestore, nWords );
		}
		else if ( pTypeDesc->fieldSize > 1 )
		{
			// Pointer array: resolved here from the saved ids.
			CUtlVector<int> ids;
			ReadIds( pRestore, nWords, ids );
			int nStride = IsPairLayout( ids ) ? 2 : 1;
			void **ppSlots = (void **)fieldInfo.pField;
			for ( int i = 0; i < pTypeDesc->fieldSize; ++i )
			{
				ppSlots[i] = ( i * nStride < ids.Count() ) ? ResolveId( ids[i * nStride] ) : NULL;
			}
			++m_nArraysHandled;
		}
		else
		{
			m_pPointerSlot = fieldInfo.pField;
			pOps->Restore( fieldInfo, pRestore );
		}

		m_pPointerSlot = pPrevSlot;
	}

	//---------------------------------

	virtual void PreSave( CSaveRestoreData * ) 
	{
		m_nSavedCount = 0;
		m_nWorldObjectId = 0;
		m_SaveIds.RemoveAll();
		ResetIdentityState();
	}
	
	//---------------------------------

	virtual void Save( ISave *pSave ) 
	{
		// The world object gets the first id.
		m_nWorldObjectId = SaveIdFor( g_PhysWorldObject );
		m_nSavedCount = m_QueuedSaves.Count();

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

		DevMsg( "Physics save: %d id(s) for %d identity write(s), %d array(s), %d object list(s)\n",
				m_SaveIds.Count(), m_nIdentitiesWritten, m_nArraysHandled, m_nListsHandled );
	}
	
	//---------------------------------

	virtual void WriteSaveHeaders( ISave *pSave )
	{
		pSave->WriteShort( &PHYS_SAVE_RESTORE_VERSION );
		pSave->WriteInt( &m_nSavedCount );
		pSave->WriteInt( &m_nWorldObjectId );
	}
	
	//---------------------------------

	virtual void PostSave() 
	{
		m_QueuedSaves.Purge();
		m_SaveIds.RemoveAll();
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
		// An unknown version can't be restored; reject the load.
		short version = pRestore->ReadShort();
		m_nLoadVersion = version;
		m_fDoLoad = ( version >= PHYS_SAVE_RESTORE_OLDEST_VERSION && version <= PHYS_SAVE_RESTORE_VERSION );
		m_bLegacyLayout = ( version < 8 );
		m_nSavedCount = 0;
		m_nWorldObjectId = 0;

		if ( !m_fDoLoad )
		{
			RejectPhysicsSave( CFmtStr( "Its physics data is version %d; this build reads versions %d to %d.\n"
										"The save was written by an incompatible build of this mod or another game. "
										"Start a new game or load a save made by this build.",
										(int)version, PHYS_SAVE_RESTORE_OLDEST_VERSION, (int)PHYS_SAVE_RESTORE_VERSION ) );
			return;
		}

		if ( version == 5 )
		{
			PhysBlockHeader_t header;
			pRestore->ReadAll( &header );
			m_nSavedCount = header.nSaved;
			m_nWorldObjectId = header.pWorldObject;
		}
		else if ( version <= 7 )
		{
			// Raw 8 byte world object pointer; its low half is its id.
			uint64 nWorldObject = 0;
			pRestore->ReadData( (char *)&m_nSavedCount, sizeof( m_nSavedCount ), sizeof( m_nSavedCount ) );
			pRestore->ReadData( (char *)&nWorldObject, sizeof( nWorldObject ), sizeof( nWorldObject ) );
			m_nWorldObjectId = (int)(unsigned int)nWorldObject;
		}
		else
		{
			pRestore->ReadInt( &m_nSavedCount );
			pRestore->ReadInt( &m_nWorldObjectId );
		}
	}

	//---------------------------------
	
	virtual void Restore( IRestore *pRestore, bool ) 
	{
		if ( !m_fDoLoad )
			return;

		if ( m_nLoadVersion == 7 && !SkipRepairTable( pRestore ) )
			return;

		ResetIdentityState();
		m_RestoredObjects.RemoveAll();

		if ( physenv )
		{
			physprerestoreparams_t params;
			params.recreatedObjectCount = 1;
			params.recreatedObjectList[0].pNewObject = g_PhysWorldObject;
			params.recreatedObjectList[0].pOldObject = IdentityKey( m_nWorldObjectId );
			physenv->PreRestore( params );
		}
		if ( m_nWorldObjectId )
		{
			m_RestoredObjects.InsertOrReplace( (unsigned int)m_nWorldObjectId, g_PhysWorldObject );
		}

		PhysObjectHeader_t header;

		int nSaved = m_nSavedCount;
		while ( nSaved-- > 0 )
		{
			pRestore->ReadAll( &header );
			pRestore->StartBlock();
			
			if ( header.hEntity != NULL )
			{
				RestoreBlock( pRestore, header );
			}

			pRestore->EndBlock();
		}

		DevMsg( "Physics restore (version %d): %d object(s), %d identity read(s), %d array(s), %d object list(s)"
				", %d duplicate id(s), %d list entr(ies) unresolved, %d lost to the old layout\n",
				m_nLoadVersion, m_RestoredObjects.Count(), m_nIdentitiesRead, m_nArraysHandled, m_nListsHandled,
				m_nDuplicateIds, m_nListUnresolved, m_nListLegacyLost );
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
		m_RestoredObjects.RemoveAll();
		ResetIdentityState();
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

			// CPhysicsEnvironment::Save writes params.pObject; RemapIntWrite turns it into an id.
			++m_nSaveDepth;
			m_pIdentitySlot = &params.pObject;
			physenv->Save( params );
			m_pIdentitySlot = NULL;
			--m_nSaveDepth;
		}
	}
	
	//---------------------------------
	
	void RestorePhysicsObject( IRestore *pRestore, const PhysObjectHeader_t &header, void **ppObject, const CPhysCollide *pCollide = NULL )
	{
		if ( physenv )
		{
			physrestoreparams_t params = { pRestore, ppObject, header.type, header.hEntity.Get(), STRING(header.modelName), pCollide, physenv, physgametrace };

			// Record id -> new object for the object lists.
			++m_nRestoreDepth;
			m_bExpectObjectIdentity = true;
			physenv->Restore( params );
			bool bReadIdentity = !m_bExpectObjectIdentity;
			m_bExpectObjectIdentity = false;
			--m_nRestoreDepth;

			if ( bReadIdentity && *ppObject )
			{
				if ( m_RestoredObjects.Find( m_nLastObjectIdentity ) != m_RestoredObjects.InvalidIndex() )
				{
					++m_nDuplicateIds;
				}
				m_RestoredObjects.InsertOrReplace( m_nLastObjectIdentity, *ppObject );
			}
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
	bool										m_fDoLoad;

	//---------------------------------
	
	CUtlMap<IPhysicsObject *, int>					m_PhysObjectModels;
	CUtlMap<IPhysicsObject *, const CPhysCollide *>	m_PhysObjectCustomModels;
	CUtlMap<const CPhysCollide *, BBox_t>			m_PhysCollideBBoxModels;

	//---------------------------------
	
	// Header values.
	int											m_nSavedCount;
	int											m_nWorldObjectId;
	int											m_nLoadVersion;
	bool										m_bLegacyLayout;	// versions 5-7 array/list layout

	//---------------------------------
	// Pointer ids

	static bool IsObjectListField( const typedescription_t *pTypeDesc )
	{
		return pTypeDesc->fieldSize == 1 && pTypeDesc->fieldName && !V_strcmp( pTypeDesc->fieldName, VPHYS_OBJECT_LIST_FIELD );
	}

	// vphysics's map key for an id.
	static void *IdentityKey( int nId )
	{
		return (void *)(uintp)(unsigned int)nId;
	}

	// Dense ids starting at 1; 0 is NULL.
	int SaveIdFor( const void *pPointer )
	{
		if ( !pPointer )
			return 0;

		int i = m_SaveIds.Find( pPointer );
		if ( i != m_SaveIds.InvalidIndex() )
			return m_SaveIds[i];

		int nId = m_SaveIds.Count() + 1;
		m_SaveIds.Insert( pPointer, nId );
		return nId;
	}

	// vphysics's own list restore allocates 4 bytes per element, so the list is filled here.
	// Both modules allocate through tier0, so vphysics can free the vector.
	void RestoreObjectList( CUtlVector<void *> *pList, IRestore *pRestore, int nWords )
	{
		pList->RemoveAll();
		++m_nListsHandled;

		if ( nWords < 1 )
			return;

		int nCount = 0;
		pRestore->ReadInt( &nCount );

		CUtlVector<int> ids;
		ReadIds( pRestore, clamp( nCount, 0, nWords - 1 ), ids );
		int nStride = IsPairLayout( ids ) ? 2 : 1;

		for ( int i = 0; i < ids.Count(); i += nStride )
		{
			void *pObject = ResolveId( ids[i] );
			if ( pObject )
			{
				pList->AddToTail( pObject );
			}
			else
			{
				// vphysics ignores NULL entries too.
				++m_nListUnresolved;
			}
		}

		if ( nStride == 2 )
		{
			m_nListLegacyLost += ids.Count() / 2;
		}
	}

	void ReadIds( IRestore *pRestore, int nCount, CUtlVector<int> &ids )
	{
		ids.SetCount( MAX( nCount, 0 ) );
		for ( int i = 0; i < ids.Count(); ++i )
		{
			ids[i] = 0;
			pRestore->ReadInt( &ids[i] );
		}
	}

	void *ResolveId( int nId )
	{
		int i = m_RestoredObjects.Find( (unsigned int)nId );
		return ( nId && i != m_RestoredObjects.InvalidIndex() ) ? m_RestoredObjects[i] : NULL;
	}

	// Whether saved ids are (low, high) pointer pairs. 32 bit version 5 saves hold one id per
	// pointer; x64 ones hold pairs, where only the even (low) words resolve.
	bool IsPairLayout( const CUtlVector<int> &ids )
	{
		if ( !m_bLegacyLayout )
			return false;
		if ( m_nLoadVersion >= 6 )
			return true;

		bool bEvenResolves = false;
		for ( int i = 0; i < ids.Count(); ++i )
		{
			if ( !ResolveId( ids[i] ) )
				continue;
			if ( i & 1 )
				return false;
			bEvenResolves = true;
		}
		return bEvenResolves;
	}

	// Skips the version 7 repair table.
	bool SkipRepairTable( IRestore *pRestore )
	{
		int nEntries = 0;
		pRestore->ReadInt( &nEntries );

		if ( nEntries < 0 || nEntries > MAX_PHYS_POINTER_REPAIRS )
		{
			m_fDoLoad = false;
			RejectPhysicsSave( CFmtStr( "Its physics data is corrupt: the repair table claims %d entries (maximum %d).",
										nEntries, (int)MAX_PHYS_POINTER_REPAIRS ) );
			return false;
		}

		for ( int i = 0; i < nEntries * 2; ++i )
		{
			int nDiscard;
			pRestore->ReadInt( &nDiscard );
		}
		return true;
	}

	void ResetIdentityState()
	{
		m_nSaveDepth = 0;
		m_nRestoreDepth = 0;
		m_pIdentitySlot = NULL;
		m_pPointerSlot = NULL;
		m_bExpectObjectIdentity = false;
		m_nLastObjectIdentity = 0;
		m_nIdentitiesWritten = 0;
		m_nIdentitiesRead = 0;
		m_nArraysHandled = 0;
		m_nListsHandled = 0;
		m_nDuplicateIds = 0;
		m_nListUnresolved = 0;
		m_nListLegacyLost = 0;
	}

	CUtlMap<const void *, int>					m_SaveIds;			// save: pointer -> id
	CUtlMap<unsigned int, void *>				m_RestoredObjects;	// restore: id -> new object

	int											m_nSaveDepth;		// > 0 inside physenv->Save()
	int											m_nRestoreDepth;	// > 0 inside physenv->Restore()
	const void									*m_pIdentitySlot;	// &params.pObject of the current save
	void										*m_pPointerSlot;	// pointer field being saved/restored
	bool										m_bExpectObjectIdentity;
	unsigned int								m_nLastObjectIdentity;

	// Diagnostics for the save/restore summary.
	int											m_nIdentitiesWritten;
	int											m_nIdentitiesRead;
	int											m_nArraysHandled;
	int											m_nListsHandled;
	int											m_nDuplicateIds;
	int											m_nListUnresolved;
	int											m_nListLegacyLost;
};

//-----------------------------------------------------------------------------

CPhysSaveRestoreBlockHandler g_PhysSaveRestoreBlockHandler;

IPhysSaveRestoreManager *g_pPhysSaveRestoreManager = &g_PhysSaveRestoreBlockHandler;

//-------------------------------------

ISaveRestoreBlockHandler *GetPhysSaveRestoreBlockHandler()
{
	return &g_PhysSaveRestoreBlockHandler;
}

// Hooks for CSave/CRestore; inert unless vphysics is saving or restoring.

bool PhysicsSaveRestoreRemapIntWrite( const int *pValue, int *pOut )
{
	return g_PhysSaveRestoreBlockHandler.RemapIntWrite( pValue, pOut );
}

void PhysicsSaveRestoreFixupIntRead( int *pValue, int nElems )
{
	g_PhysSaveRestoreBlockHandler.FixupIntRead( pValue, nElems );
}

void PhysicsSaveRestoreSaveCustomField( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave )
{
	g_PhysSaveRestoreBlockHandler.SaveCustomField( fieldInfo, pSave );
}

void PhysicsSaveRestoreRestoreCustomField( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore, int nBytes )
{
	g_PhysSaveRestoreBlockHandler.RestoreCustomField( fieldInfo, pRestore, nBytes );
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
