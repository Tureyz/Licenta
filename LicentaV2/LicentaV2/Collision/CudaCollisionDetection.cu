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


///https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/master/PositionBasedDynamics/PositionBasedDynamics.cpp
__device__ void DeformableUtils::baryVF(const float3 & p, const float3 & p0, const float3 & p1, const float3 & p2, float & b0, float & b1, float & b2)
{
	b0 = b1 = b2 = 1.f / 3;
	
	float3 d1 = p1 - p0;
	float3 d2 = p2 - p0;
	float3 pp0 = p - p0;

	float a = CudaUtils::dot(d1, d1);
	float b = CudaUtils::dot(d2, d1);
	float c = CudaUtils::dot(pp0, d1);
	float d = b;
	float e = CudaUtils::dot(d2, d2);
	float f = CudaUtils::dot(pp0, d2);

	float det = a * e - b * d;

	if (det != 0.f)
	{
		float s = (c * e - b * f) / det;
		float t = (a * f - c * d) / det;

		b0 = 1.0 - s - t;
		b1 = s;
		b2 = t;

		if (b0 < 0.f)
		{
			float3 d = p2 - p1;
			float d2 = CudaUtils::dot(d, d);
			t = d2 == 0.f ? 0.5f : CudaUtils::clamp(CudaUtils::dot(d, p - p1) / d2, 0.f, 1.f);

			b0 = 0.f;
			b1 = (1.f - t);
			b2 = t;
		}
		else if (b1 < 0.f)
		{
			float3 d = p0 - p2;
			float d2 = CudaUtils::dot(d, d);
			t = d2 == 0.f ? 0.5 : CudaUtils::clamp(CudaUtils::dot(d, p - p2) / d2, 0.f, 1.f);
			b0 = 0.f;
			b1 = (1.f - t);
			b2 = t;
		}
		else if (b2 < 0.f)
		{
			float3 d = p1 - p0;
			float d2 = CudaUtils::dot(d, d);
			t = d2 == 0.f ? 0.5 : CudaUtils::clamp(CudaUtils::dot(d, p - p0) / d2, 0.f, 1.f);
			b0 = 0.f;
			b1 = (1.f - t);
			b2 = t;
		}
	}
}

///https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/blob/master/PositionBasedDynamics/PositionBasedDynamics.cpp
__device__ void DeformableUtils::baryEE(const float3 & p0, const float3 & p1, const float3 & p2, const float3 & p3, float & s, float & t)
{
	float3 d0 = p1 - p0;
	float3 d1 = p3 - p2;

	float a = CudaUtils::dot(d0, d0);
	float b = -CudaUtils::dot(d0, d1);
	float c = -b;
	float d = -CudaUtils::dot(d1, d1);
	float e = CudaUtils::dot(p2 - p0, d0);
	float f = CudaUtils::dot(p2 - p0, d1);
	float det = a * d - b * c;

	if (det != 0.f)
	{
		det = 1.f / det;
		s = (e * d - b * f) * det;
		t = (a * f - e * c) * det;
	}
	else
	{
		float s0 = CudaUtils::dot(p0, d0);
		float s1 = CudaUtils::dot(p1, d0);
		float t0 = CudaUtils::dot(p2, d0);
		float t1 = CudaUtils::dot(p3, d0);

		bool flip0 = false;
		bool flip1 = false;

		if (s0 > s1)
		{
			float aux = s0;
			s0 = s1;
			s1 = aux;
			flip0 = true;
		}

		if (t0 > t1)
		{
			float aux = t0;
			t0 = t1;
			t1 = aux;
			flip1 = true;
		}

		if (s0 >= t1)
		{
			s = !flip0 ? 0.f : 1.f;
			t = !flip1 ? 1.f : 0.f;
		}
		else if (t0 >= s1)
		{
			s = !flip0 ? 1.f : 0.f;
			t = !flip1 ? 0.f : 1.f;
		}
		else
		{
			float mid = s0 > t0 ? (s0 + t1) * 0.5f : (t0 + s1) * 0.5f;
			s = (s0 == s1) ? 0.5f : (mid - s0) / (s1 - s0);
			t = (t0 == t1) ? 0.5f : (mid - t0) / (t1 - t0);
		}
	}

	CudaUtils::clamp(s, 0.f, 1.f);
	CudaUtils::clamp(t, 0.f, 1.f);
}
