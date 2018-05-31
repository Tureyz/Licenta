#include "CudaCollisionDetection.cuh"

#include "../Core/CubWrappers.cuh"
#include "CudaPrimitiveTests.cuh"

void DeformableUtils::CreateTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles,
	thrust::device_vector<Physics::AABBCollision> &rawAABBCols, const thrust::device_vector<bool> &flaggedAABBCols,
	thrust::device_vector<Physics::PrimitiveContact> &vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t &vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact> &eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t &eeContactsSize,
	void *&tempStorage, uint64_t &tempStorageSize)
{
	int filteredContactsSize = 0;



	//CubWrap::SelectInPlace(rawAABBCols, EqualToValue(timestamp), filteredContactsSize, tempStorage, tempStorageSize);
	CubWrap::SelectFlagged(rawAABBCols, flaggedAABBCols, rawAABBCols.size(), rawAABBCols, filteredContactsSize, tempStorage, tempStorageSize);

	//printf("Cols before: %llu, after: %d\n", rawAABBCols.size(), filteredContactsSize);

	vfContactsSize = 6 * filteredContactsSize;
	eeContactsSize = 9 * filteredContactsSize;

	if (vfContacts.size() < vfContactsSize)
	{
		vfContacts.resize(vfContactsSize);
		vfFlags.resize(vfContactsSize);
	}

	if (eeContacts.size() < eeContactsSize)
	{
		eeContacts.resize(eeContactsSize);
		eeFlags.resize(eeContactsSize);
	}

	if (filteredContactsSize == 0)
		return;


	/*if (filteredContactsSize % 256 != 0)
	printf("CONTACTSSSS\n");*/


	const int numBlocks = (filteredContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	CreateTriangleTests << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(filteredContactsSize, thrust::raw_pointer_cast(&triangles[0]), thrust::raw_pointer_cast(&rawAABBCols[0]),
		thrust::raw_pointer_cast(&vfContacts[0]), thrust::raw_pointer_cast(&eeContacts[0]));
}

__global__ void DeformableUtils::CreateTriangleTests(const int filteredContactsSize, const Physics::CudaTriangle *__restrict__ triangles,
	const Physics::AABBCollision * __restrict__ filteredCols,
	Physics::PrimitiveContact *__restrict__ vfContacts, Physics::PrimitiveContact *__restrict__ eeContacts)
{
	int id = CudaUtils::MyID();
	if (id >= filteredContactsSize)
		return;


	int myVFStart = id * 6;
	int myEEStart = id * 9;

	//printf("[%d] tri1: %d, tri2: %d\n", id, filteredCols[id].m_id1, filteredCols[id].m_id2);
	Physics::CudaTriangle tri1 = triangles[filteredCols[id].m_id1];
	Physics::CudaTriangle tri2 = triangles[filteredCols[id].m_id2];


	vfContacts[myVFStart++] = Physics::PrimitiveContact(tri1.m_v1, tri2.m_v1, tri2.m_v2, tri2.m_v3);
	vfContacts[myVFStart++] = Physics::PrimitiveContact(tri1.m_v2, tri2.m_v1, tri2.m_v2, tri2.m_v3);
	vfContacts[myVFStart++] = Physics::PrimitiveContact(tri1.m_v3, tri2.m_v1, tri2.m_v2, tri2.m_v3);

	vfContacts[myVFStart++] = Physics::PrimitiveContact(tri2.m_v1, tri1.m_v1, tri1.m_v2, tri1.m_v3);
	vfContacts[myVFStart++] = Physics::PrimitiveContact(tri2.m_v2, tri1.m_v1, tri1.m_v2, tri1.m_v3);
	vfContacts[myVFStart] = Physics::PrimitiveContact(tri2.m_v3, tri1.m_v1, tri1.m_v2, tri1.m_v3);

	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v1, tri2.m_v2);
	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v1, tri2.m_v2);
	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v1, tri2.m_v2);

	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v1, tri2.m_v3);
	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v1, tri2.m_v3);
	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v1, tri2.m_v3);

	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v2, tri2.m_v3);
	eeContacts[myEEStart++] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v2, tri2.m_v3);
	eeContacts[myEEStart] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v2, tri2.m_v3);
}

void DeformableUtils::DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle>& triangles, const thrust::device_vector<float3>& positions,
	thrust::device_vector<Physics::PrimitiveContact>& vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t & vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact>& eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t & eeContactsSize,
	const float thickness, void *&tempStorage, uint64_t &tempStorageSize)
{

	CudaPrimitiveTests::DCDTriangleTests(triangles, positions, vfContacts, vfFlags, vfContactsSize, eeContacts, eeFlags, eeContactsSize, thickness);


	//printf("before vf: %llu, ee: %llu\n", vfContactsSize, eeContactsSize);
	int vfsz = 0, eesz = 0;


	///TODO Replace Select with Flagged, check performance
	//CubWrap::SelectInPlace(vfContacts, PositiveCollisionTest(), vfsz, tempStorage, tempStorageSize);
	//CubWrap::SelectInPlace(eeContacts, PositiveCollisionTest(), eesz, tempStorage, tempStorageSize);

	CubWrap::SelectFlagged(vfContacts, vfFlags, vfContactsSize, vfContacts, vfsz, tempStorage, tempStorageSize);
	CubWrap::SelectFlagged(eeContacts, eeFlags, eeContactsSize, eeContacts, eesz, tempStorage, tempStorageSize);

	vfContactsSize = (uint64_t)vfsz;
	eeContactsSize = (uint64_t)eesz;


	//if (vfsz != 0 || eesz != 0)
	//{

	//	printf("after vf: %llu, ee: %llu\n", vfContactsSize, eeContactsSize);
	//}

}

void DeformableUtils::CCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle>& triangles,
	const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& velocities,
	thrust::device_vector<Physics::PrimitiveContact>& vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t & vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact>& eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t & eeContactsSize,
	const float thickness, const float timeStep, void *& tempStorage, uint64_t & tempStorageSize)
{
	CudaPrimitiveTests::CCDTriangleTests(triangles, positions, velocities, vfContacts, vfFlags, vfContactsSize, eeContacts, eeFlags, eeContactsSize, thickness, timeStep);

	int vfsz = 0, eesz = 0;


	///TODO Replace Select with Flagged, check performance
	//CubWrap::SelectInPlace(vfContacts, PositiveCollisionTest(), vfsz, tempStorage, tempStorageSize);
	//CubWrap::SelectInPlace(eeContacts, PositiveCollisionTest(), eesz, tempStorage, tempStorageSize);

	CubWrap::SelectFlagged(vfContacts, vfFlags, vfContactsSize, vfContacts, vfsz, tempStorage, tempStorageSize);
	CubWrap::SelectFlagged(eeContacts, eeFlags, eeContactsSize, eeContacts, eesz, tempStorage, tempStorageSize);

	vfContactsSize = (uint64_t)vfsz;
	eeContactsSize = (uint64_t)eesz;
}