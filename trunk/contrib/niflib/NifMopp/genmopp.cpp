// ***** BEGIN LICENSE BLOCK *****
//
// Copyright (c) 2006-2008, NIF File Format Library and Tools.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//    * Neither the name of the NIF File Format Library and Tools
//      project nor the names of its contributors may be used to endorse
//      or promote products derived from this software without specific
//      prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ***** END LICENSE BLOCK *****

#include "NifMopp.h"

//
// Math and base include
#include <Common/Base/hkBase.h>
#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/Memory/hkThreadMemory.h>
#include <Common/Base/Memory/Memory/Pool/hkPoolMemory.h>
#include <Common/Base/System/Error/hkDefaultError.h>
#include <Common/Base/Monitor/hkMonitorStream.h>

#include <Common/Base/System/Io/FileSystem/hkFileSystem.h>
#include <Common/Base/Container/LocalArray/hkLocalBuffer.h>
//
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics/Collide/Shape/Convex/ConvexTranslate/hkpConvexTranslateShape.h>
#include <Physics/Collide/Shape/Convex/ConvexTransform/hkpConvexTransformShape.h>
#include <Physics/Collide/Shape/Compound/Collection/SimpleMesh/hkpSimpleMeshShape.h>
#include <Physics/Collide/Shape/Compound/Collection/List/hkpListShape.h>
#include <Physics/Collide/Shape/Convex/Capsule/hkpCapsuleShape.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppBvTreeShape.h>
#include <Physics/Collide/Shape/Compound/Tree/Mopp/hkpMoppUtility.h>
#include <Physics/Internal/Collide/Mopp/Code/hkpMoppCode.h>

#pragma comment(lib, "hkBase.lib")
#pragma comment(lib, "hkSerialize.lib")
#pragma comment(lib, "hkpInternal.lib")
#pragma comment(lib, "hkpUtilities.lib")
#pragma comment(lib, "hkpCollide.lib")
#pragma comment(lib, "hkpConstraintSolver.lib")

#ifdef _MANAGED
#pragma managed(push, off)
#endif

static hkpSimpleMeshShape* ConstructHKMesh( int nVerts, Point3 const* verts, int nTris, Triangle const * tris, int vertOffset = 0)
{
	hkpSimpleMeshShape * storageMeshShape = new hkpSimpleMeshShape( 0.01f );
	hkArray<hkVector4> &vertices = storageMeshShape->m_vertices;
	hkArray<hkpSimpleMeshShape::Triangle> &triangles = storageMeshShape->m_triangles;

	triangles.setSize( 0 );
	for (int i=0;i<nTris;++i) {
		Triangle const &tri = tris[i];
		hkpSimpleMeshShape::Triangle hktri;
		hktri.m_a = tri[0] - vertOffset;
		hktri.m_b = tri[1] - vertOffset;
		hktri.m_c = tri[2] - vertOffset;
		triangles.pushBack( hktri );
	}

	vertices.setSize( 0 );
	for (int i=0;i<nVerts;++i) {
		Point3 const &vert = verts[i];
		vertices.pushBack( hkVector4(vert.x, vert.y, vert.z) );
	}
	//storageMeshShape->setRadius(1.0f);
	return storageMeshShape;
}

static hkpMoppCode* k_phkpMoppCode = NULL;

extern void CloseHavokMopp()
{
	if (k_phkpMoppCode)
	{
		k_phkpMoppCode->removeReference();
		k_phkpMoppCode = NULL;
	}
}


static int InternalGenerateCode(int nVerts, Point3 const* verts, int nTris, Triangle const *tris)
{
	int retCode = 0;
	InitializeHavok();

	if (k_phkpMoppCode)
	{
		k_phkpMoppCode->removeReference();
		k_phkpMoppCode = NULL;
	}

	hkpShapeCollection * list = ConstructHKMesh(nVerts, verts, nTris, tris);

	hkpMoppCompilerInput mfr;
	mfr.setAbsoluteFitToleranceOfAxisAlignedTriangles( hkVector4( 0.1f, 0.1f, 0.1f ) );
	//mfr.setAbsoluteFitToleranceOfTriangles(0.1f);
	//mfr.setAbsoluteFitToleranceOfInternalNodes(0.0001f);

	k_phkpMoppCode = hkpMoppUtility::buildCode(list, mfr);

	list->removeReference();

	return k_phkpMoppCode->m_data.getSize();
}
static int InternalGenerateCodeWithSubshapes(int nShapes, int *subShapes, int nVerts, Point3 const* verts, int nTris, Triangle const *tris)
{
	int retCode = 0;
	InitializeHavok();

	if (k_phkpMoppCode)
	{
		k_phkpMoppCode->removeReference();
		k_phkpMoppCode = NULL;
	}

	//int voff = 0;
	//int toff = 0;

	//hkpShape** shapes = new hkpShape*[nShapes];

	//for (int i = 0; i<nShapes; ++i) {
	//	int vend = (voff + subShapes[i] );
	//	int tend = toff;
	//	while ( tend < nTris ) {
	//		Triangle const & t = tris[tend];
	//		if ( t.a >= vend || t.b >= vend || t.c >= vend )
	//			break;
	//		tend++;
	//	}
	//	hkpSimpleMeshShape* shape = ConstructHKMesh(vend-voff, &verts[voff], tend-toff, &tris[toff], voff);
	//	shapes[i] = shape;
 //     voff = vend;
 //     toff = tend;
	//}

	//hkpListShape* list = new hkpListShape(&shapes[0], nShapes);

	hkpSimpleMeshShape* list = ConstructHKMesh(nVerts, verts, nTris, tris);
   list->setRadius(0.1000f);

	hkArray<hkUint8> &materialIndices = list->m_materialIndices;

	materialIndices.setSize(nShapes);
	for (int i = 0; i<nShapes; ++i)
		materialIndices[i] = subShapes[i];

	hkpMoppCompilerInput mfr;
	mfr.setAbsoluteFitToleranceOfAxisAlignedTriangles( hkVector4( 0.1945f, 0.1945f, 0.1945f ) );
	mfr.setAbsoluteFitToleranceOfTriangles(0.1945f);
	mfr.setAbsoluteFitToleranceOfInternalNodes(0.3f);

	k_phkpMoppCode = hkpMoppUtility::buildCode(list, mfr);

	//for (int i = 0; i<nShapes; ++i) {
	//	shapes[i]->removeReference();
	//}
	//delete [] shapes;

	list->removeReference();

	return k_phkpMoppCode->m_data.getSize();
}

extern "C" 
int __stdcall GenerateMoppCodeWithSubshapes(int nShapes, int *shapeVerts, int nVerts, Point3 const* verts, int nTris, Triangle const *tris)
{
	int retcode = 0;
	__try
	{
		if (nShapes <= 1)
			retcode = InternalGenerateCode(nVerts, verts, nTris, tris);
		else
			retcode = InternalGenerateCodeWithSubshapes(nShapes, shapeVerts, nVerts, verts, nTris, tris);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
		retcode = -1;
	}
	return retcode;
}

extern "C" 
int __stdcall GenerateMoppCode(int nVerts, Point3 const* verts, int nTris, Triangle const *tris)
{
	int retcode = 0;
	__try
	{
		retcode = InternalGenerateCode(nVerts, verts, nTris, tris);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
		retcode = -1;
	}
	return retcode;
}

extern "C" 
int __stdcall RetrieveMoppCode(int nBuffer, hkUint8 *buffer)
{
	if ( k_phkpMoppCode == NULL )
		return 0;

	if ( nBuffer != k_phkpMoppCode->m_data.getSize() )
		return -(k_phkpMoppCode->m_data.getSize());

	memcpy(buffer, &k_phkpMoppCode->m_data[0], nBuffer);
	return k_phkpMoppCode->m_data.getSize();
}

extern "C" 
int __stdcall RetrieveMoppScale(float *value)
{
	if ( k_phkpMoppCode == NULL )
		return 0;

	if (value == NULL)
		return 0;
	
	*value = k_phkpMoppCode->m_info.getScale();
	return 1;
}

extern "C" 
int __stdcall RetrieveMoppOrigin( float value[3] )
{
	if ( k_phkpMoppCode == NULL )
		return 0;

	if ( value == NULL )
		return 0;

	value[0] = k_phkpMoppCode->m_info.m_offset(0);
	value[1] = k_phkpMoppCode->m_info.m_offset(1);
	value[2] = k_phkpMoppCode->m_info.m_offset(2);
	return 1;
}

#ifdef _MANAGED
#pragma managed(pop)
#endif

