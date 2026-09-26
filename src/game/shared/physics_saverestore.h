//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//=============================================================================//

#ifndef PHYSICS_SAVERESTORE_H
#define PHYSICS_SAVERESTORE_H

#if defined( _WIN32 )
#pragma once
#endif

#include "vphysics_interface.h"

class ISaveRestoreBlockHandler;
class IPhysicsObject;
class CPhysCollide;

//-----------------------------------------------------------------------------

ISaveRestoreBlockHandler *GetPhysSaveRestoreBlockHandler();
ISaveRestoreOps *GetPhysObjSaveRestoreOps( PhysInterfaceId_t );

//-------------------------------------

#define DEFINE_PHYSPTR(name) \
	{ FIELD_CUSTOM, #name, { (int)offsetof(classNameTypedef,name), 0 }, 1, FTYPEDESC_SAVE, NULL, GetPhysObjSaveRestoreOps( GetPhysIID( &(((classNameTypedef *)0)->name) ) ), NULL }

#define DEFINE_PHYSPTR_ARRAY(name) \
	{ FIELD_CUSTOM, #name, { (int)offsetof(classNameTypedef,name), 0 }, ARRAYSIZE(((classNameTypedef *)0)->name), FTYPEDESC_SAVE, NULL, GetPhysObjSaveRestoreOps( GetPhysIID( &(((classNameTypedef *)0)->name[0]) ) ), NULL }

//-----------------------------------------------------------------------------

abstract_class IPhysSaveRestoreManager
{
public:
	virtual void NoteBBox( const Vector &mins, const Vector &maxs, CPhysCollide * ) = 0;

	virtual void AssociateModel( IPhysicsObject *, int modelIndex ) = 0;
	virtual void AssociateModel( IPhysicsObject *, const CPhysCollide *pModel ) = 0;
	virtual void ForgetModel( IPhysicsObject * ) = 0;

	virtual void ForgetAllModels() = 0;
};

extern IPhysSaveRestoreManager *g_pPhysSaveRestoreManager;

// x64 vphysics pointer id hooks (see physics_saverestore.cpp).
struct SaveRestoreFieldInfo_t;
class ISave;
class IRestore;

// CSave::WriteInt: true if *pValue is a pointer, with its id in *pOut.
bool PhysicsSaveRestoreRemapIntWrite( const int *pValue, int *pOut );
// CRestore::ReadInt: zero extends a pointer id vphysics just read.
void PhysicsSaveRestoreFixupIntRead( int *pValue, int nElems );
// FIELD_CUSTOM save/restore; nBytes is the saved size of the field.
void PhysicsSaveRestoreSaveCustomField( const SaveRestoreFieldInfo_t &fieldInfo, ISave *pSave );
void PhysicsSaveRestoreRestoreCustomField( const SaveRestoreFieldInfo_t &fieldInfo, IRestore *pRestore, int nBytes );

//=============================================================================

#endif // PHYSICS_SAVERESTORE_H
