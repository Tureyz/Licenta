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

	cudaMemset(cu::raw(vfFlags), 1, vfContactsSize * sizeof(bool));
	cudaMemset(cu::raw(eeFlags), 1, eeContactsSize * sizeof(bool));


	const int numBlocks = (filteredContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	CreateTriangleTests << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(filteredContactsSize, cu::raw(triangles), cu::raw(rawAABBCols),
		cu::raw(vfContacts), cu::raw(vfFlags), cu::raw(eeContacts), cu::raw(eeFlags));
}

__global__ void DeformableUtils::CreateTriangleTests(const int filteredContactsSize, const Physics::CudaTriangle *__restrict__ triangles,
	const Physics::AABBCollision * __restrict__ filteredCols,
	Physics::PrimitiveContact *__restrict__ vfContacts, bool * __restrict__ vfFlags, Physics::PrimitiveContact *__restrict__ eeContacts, bool * __restrict__ eeFlags)
{
	int id = CudaUtils::MyID();
	if (id >= filteredContactsSize)
		return;


	int myVFStart = id * 6;
	int myEEStart = id * 9;

	//printf("[%d] tri1: %d, tri2: %d\n", id, filteredCols[id].m_id1, filteredCols[id].m_id2);
	Physics::CudaTriangle tri1 = triangles[filteredCols[id].m_id1];
	Physics::CudaTriangle tri2 = triangles[filteredCols[id].m_id2];


	//vfContacts[myVFStart] = Physics::PrimitiveContact(tri1.m_v1, tri2.m_v1, tri2.m_v2, tri2.m_v3);
	//vfContacts[myVFStart + 1] = Physics::PrimitiveContact(tri1.m_v2, tri2.m_v1, tri2.m_v2, tri2.m_v3);
	//vfContacts[myVFStart + 2] = Physics::PrimitiveContact(tri1.m_v3, tri2.m_v1, tri2.m_v2, tri2.m_v3);
	//vfContacts[myVFStart + 3] = Physics::PrimitiveContact(tri2.m_v1, tri1.m_v1, tri1.m_v2, tri1.m_v3);
	//vfContacts[myVFStart + 4] = Physics::PrimitiveContact(tri2.m_v2, tri1.m_v1, tri1.m_v2, tri1.m_v3);
	//vfContacts[myVFStart + 5] = Physics::PrimitiveContact(tri2.m_v3, tri1.m_v1, tri1.m_v2, tri1.m_v3);

	//eeContacts[myEEStart] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v1, tri2.m_v2);
	//eeContacts[myEEStart + 1] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v1, tri2.m_v2);
	//eeContacts[myEEStart + 2] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v1, tri2.m_v2);
	//eeContacts[myEEStart + 3] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v1, tri2.m_v3);
	//eeContacts[myEEStart + 4] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v1, tri2.m_v3);
	//eeContacts[myEEStart + 5] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v1, tri2.m_v3);
	//eeContacts[myEEStart + 6] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v2, tri2.m_v3);
	//eeContacts[myEEStart + 7] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v2, tri2.m_v3);
	//eeContacts[myEEStart + 8] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v2, tri2.m_v3);

	//if (AdjacentTriangles(tri1, tri2))
	//{
	//	for (int i = 0; i < 6; ++i)
	//	{
	//		vfFlags[myVFStart + i] = false;
	//	}

	//	for (int i = 0; i < 9; ++i)
	//	{
	//		eeFlags[myEEStart + i] = false;
	//	}
	//}
	//else
	//{
		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 5))
			vfContacts[myVFStart] = Physics::PrimitiveContact(tri1.m_v1, tri2.m_v1, tri2.m_v2, tri2.m_v3);
		else
			vfFlags[myVFStart] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 4))
			vfContacts[myVFStart + 1] = Physics::PrimitiveContact(tri1.m_v2, tri2.m_v1, tri2.m_v2, tri2.m_v3);
		else
			vfFlags[myVFStart + 1] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 3))
			vfContacts[myVFStart + 2] = Physics::PrimitiveContact(tri1.m_v3, tri2.m_v1, tri2.m_v2, tri2.m_v3);
		else
			vfFlags[myVFStart + 2] = false;



		if (CudaUtils::GetBit(tri2.m_assignedFeatures, 5))
			vfContacts[myVFStart + 3] = Physics::PrimitiveContact(tri2.m_v1, tri1.m_v1, tri1.m_v2, tri1.m_v3);
		else
			vfFlags[myVFStart + 3] = false;

		if (CudaUtils::GetBit(tri2.m_assignedFeatures, 4))
			vfContacts[myVFStart + 4] = Physics::PrimitiveContact(tri2.m_v2, tri1.m_v1, tri1.m_v2, tri1.m_v3);
		else
			vfFlags[myVFStart + 4] = false;

		if (CudaUtils::GetBit(tri2.m_assignedFeatures, 3))
			vfContacts[myVFStart + 5] = Physics::PrimitiveContact(tri2.m_v3, tri1.m_v1, tri1.m_v2, tri1.m_v3);
		else
			vfFlags[myVFStart + 5] = false;







		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 2) & CudaUtils::GetBit(tri2.m_assignedFeatures, 2))
			eeContacts[myEEStart] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v1, tri2.m_v2);
		else
			eeFlags[myEEStart] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 0) & CudaUtils::GetBit(tri2.m_assignedFeatures, 2))
			eeContacts[myEEStart + 1] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v1, tri2.m_v2);
		else
			eeFlags[myEEStart + 1] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 1) & CudaUtils::GetBit(tri2.m_assignedFeatures, 2))
			eeContacts[myEEStart + 2] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v1, tri2.m_v2);
		else
			eeFlags[myEEStart + 2] = false;



		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 2) & CudaUtils::GetBit(tri2.m_assignedFeatures, 0))
			eeContacts[myEEStart + 3] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v1, tri2.m_v3);
		else
			eeFlags[myEEStart + 3] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 0) & CudaUtils::GetBit(tri2.m_assignedFeatures, 0))
			eeContacts[myEEStart + 4] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v1, tri2.m_v3);
		else
			eeFlags[myEEStart + 4] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 1) & CudaUtils::GetBit(tri2.m_assignedFeatures, 0))
			eeContacts[myEEStart + 5] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v1, tri2.m_v3);
		else
			eeFlags[myEEStart + 5] = false;



		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 2) & CudaUtils::GetBit(tri2.m_assignedFeatures, 1))
			eeContacts[myEEStart + 6] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v2, tri2.m_v2, tri2.m_v3);
		else
			eeFlags[myEEStart + 6] = false;


		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 0) & CudaUtils::GetBit(tri2.m_assignedFeatures, 1))
			eeContacts[myEEStart + 7] = Physics::PrimitiveContact(tri1.m_v1, tri1.m_v3, tri2.m_v2, tri2.m_v3);
		else
			eeFlags[myEEStart + 7] = false;

		if (CudaUtils::GetBit(tri1.m_assignedFeatures, 1) & CudaUtils::GetBit(tri2.m_assignedFeatures, 1))
			eeContacts[myEEStart + 8] = Physics::PrimitiveContact(tri1.m_v2, tri1.m_v3, tri2.m_v2, tri2.m_v3);
		else
			eeFlags[myEEStart + 8] = false;
	//}
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

void DeformableUtils::CCDTriangleTests(const FeatureList::FeatureList &features, const thrust::device_vector<Physics::CudaTriangle>& triangles,
	const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& prevPositions, const thrust::device_vector<float3>& velocities,
	thrust::device_vector<Physics::PrimitiveContact>& vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t & vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact>& eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t & eeContactsSize,
	const float thickness, const float timeStep, void *& tempStorage, uint64_t & tempStorageSize)
{

	///TODO culling goes here


	if (vfContactsSize + eeContactsSize == 0)
		return;

	int vfsz = 0, eesz = 0;

	int numBlocks;



	numBlocks = cu::nb(vfContactsSize + eeContactsSize);


	CudaPrimitiveTests::FeatureAABBTests << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (positions.size(), cu::raw(features.m_particleAABBMins), cu::raw(features.m_particleAABBMaxs),
		cu::raw(features.m_edgeMap), cu::raw(features.m_edgev1s), cu::raw(features.m_edgev2s), cu::raw(features.m_edgeAABBMins), cu::raw(features.m_edgeAABBMaxs), NULL,
		NULL, NULL, cu::raw(vfContacts), cu::raw(vfFlags), vfContactsSize, cu::raw(eeContacts), cu::raw(eeFlags), eeContactsSize);


	//CubWrap::SelectFlagged(vfContacts, vfFlags, vfContactsSize, vfContacts, vfsz, tempStorage, tempStorageSize);
	//CubWrap::SelectFlagged(eeContacts, eeFlags, eeContactsSize, eeContacts, eesz, tempStorage, tempStorageSize);

	//vfContactsSize = (uint64_t)vfsz;
	//eeContactsSize = (uint64_t)eesz;

	//numBlocks = cu::nb(vfContactsSize + eeContactsSize);

	//DeformingNonPenetrationFilters << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (cu::raw(positions), cu::raw(prevPositions), cu::raw(vfContacts), cu::raw(vfFlags), vfContactsSize,
	//	cu::raw(eeContacts), cu::raw(eeFlags), eeContactsSize, thickness, timeStep);


	///........................

	CubWrap::SelectFlagged(vfContacts, vfFlags, vfContactsSize, vfContacts, vfsz, tempStorage, tempStorageSize);
	CubWrap::SelectFlagged(eeContacts, eeFlags, eeContactsSize, eeContacts, eesz, tempStorage, tempStorageSize);

	vfContactsSize = (uint64_t)vfsz;
	eeContactsSize = (uint64_t)eesz;

	CudaPrimitiveTests::CCDTriangleTests(triangles, prevPositions, velocities, vfContacts, vfFlags, vfContactsSize, eeContacts, eeFlags, eeContactsSize, thickness, timeStep);



	///TODO Replace Select with Flagged, check performance
	//CubWrap::SelectInPlace(vfContacts, PositiveCollisionTest(), vfsz, tempStorage, tempStorageSize);
	//CubWrap::SelectInPlace(eeContacts, PositiveCollisionTest(), eesz, tempStorage, tempStorageSize);

	CubWrap::SelectFlagged(vfContacts, vfFlags, vfContactsSize, vfContacts, vfsz, tempStorage, tempStorageSize);
	CubWrap::SelectFlagged(eeContacts, eeFlags, eeContactsSize, eeContacts, eesz, tempStorage, tempStorageSize);

	vfContactsSize = (uint64_t)vfsz;
	eeContactsSize = (uint64_t)eesz;
}

__device__ bool DeformableUtils::AdjacentTriangles(const Physics::CudaTriangle & a, Physics::CudaTriangle & b)
{
	return a.m_v1 == b.m_v1 || a.m_v1 == b.m_v2 || a.m_v1 == b.m_v3 || a.m_v2 == b.m_v1 || a.m_v2 == b.m_v2 || a.m_v3 == b.m_v3 || a.m_v3 == b.m_v1 || a.m_v3 == b.m_v2 || a.m_v3 == b.m_v3;
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

__global__ void DeformableUtils::DeformingNonPenetrationFilters(const float3 *__restrict__ positions, const float3 * __restrict__ prevPositions,
	const Physics::PrimitiveContact *__restrict__ vfContacts, bool *__restrict__ vfFlags, const uint64_t vfSize,
	const Physics::PrimitiveContact *__restrict__ eeContacts, bool *__restrict__ eeFlags, const uint64_t eeSize,
	const float thickness, const float timestep)
{
	const int id = CudaUtils::MyID();

	if (id >= vfSize + eeSize)
		return;

	if (id < vfSize)
	{
		DeformingNonPenetrationFilterVF(id, positions, prevPositions, vfContacts, vfFlags, thickness, timestep);
	}
	else 
	{
		DeformingNonPenetrationFilterEE(id - vfSize, positions, prevPositions, eeContacts, eeFlags, thickness, timestep);
	}
}

__device__ void DeformableUtils::DeformingNonPenetrationFilterVF(const int id, const float3 *__restrict__ positions, const float3 * __restrict__ prevPositions,
	const Physics::PrimitiveContact *__restrict__ vfContacts, bool *__restrict__ vfFlags,
	const float thickness, const float timestep)
{
	if (!vfFlags[id])
		return;

	const Physics::PrimitiveContact myContact = vfContacts[id];

	float3 p0 = prevPositions[myContact.v1];
	float3 a0 = prevPositions[myContact.v2];
	float3 b0 = prevPositions[myContact.v3];
	float3 c0 = prevPositions[myContact.v4];

	float3 p1 = positions[myContact.v1];
	float3 a1 = positions[myContact.v2];
	float3 b1 = positions[myContact.v3];
	float3 c1 = positions[myContact.v4];

	float3 n0 = CudaUtils::cross(b0 - a0, c0 - a0);
	float3 n1 = CudaUtils::cross(b1 - a1, c1 - a1);

	float3 va = a1 - a0;
	float3 vb = b1 - b0;
	float3 vc = c1 - c0;

	float3 n = (n0 + n1 - CudaUtils::cross(vb - va, vc - va)) / 2.f;

	float3 p0a0 = p0 - a0;
	float3 p1a1 = p1 - a1;

	p0a0 *= 1.f + (thickness / CudaUtils::len(p0a0));
	p1a1 *= 1.f + (thickness / CudaUtils::len(p1a1));

	float A = CudaUtils::dot(p0a0, n0), B = CudaUtils::dot(p1a1, n1);
	float C = CudaUtils::dot(p0a0, n), D = CudaUtils::dot(p1a1, n);
	float E = CudaUtils::dot(p0a0, n1), F = CudaUtils::dot(p1a1, n0);

	float term3 = (2 * C + F) * 0.3333333f;
	float term4 = (2 * D + E) * 0.3333333f;

	//vfFlags[id] = ~((A < 2 * thickness && B < 2 * thickness && term3 < 2 * thickness && term4 < 2 * thickness) || (A >= 2 * thickness && B >= 2 * thickness && term3 >= 2 * thickness && term4 >= 2 * thickness));
	vfFlags[id] = ~((A < 0 && B < 0 && term3 < 0 && term4 < 0) || (A >= 0 && B >= 0 && term3 >= 0 && term4 >= 0));

}

__device__ void DeformableUtils::DeformingNonPenetrationFilterEE(const int id, const float3 *__restrict__ positions, const float3 * __restrict__ prevPositions,
	const Physics::PrimitiveContact *__restrict__ eeContacts, bool *__restrict__ eeFlags,
	const float thickness, const float timestep)
{
	if (!eeFlags[id])
		return;

	const Physics::PrimitiveContact myContact = eeContacts[id];

	float3 u0 = prevPositions[myContact.v1];
	float3 v0 = prevPositions[myContact.v2];
	float3 k0 = prevPositions[myContact.v3];
	float3 l0 = prevPositions[myContact.v4];

	float3 u1 = positions[myContact.v1];
	float3 v1 = positions[myContact.v2];
	float3 k1 = positions[myContact.v3];
	float3 l1 = positions[myContact.v4];

	float3 n0 = CudaUtils::cross(u0 - k0, v0 - k0);
	float3 n1 = CudaUtils::cross(u1 - k1, v1 - k1);

	float3 vk = k1 - k0;
	float3 vu = u1 - u0;
	float3 vv = v1 - v0;

	float3 n = (n0 + n1 - CudaUtils::cross(vu - vk, vv - vk)) / 2.f;

	float3 l0k0 = l0 - k0;
	float3 l1k1 = l1 - k1;

	l0k0 *= 1.f + (thickness / CudaUtils::len(l0k0));
	l1k1 *= 1.f + (thickness / CudaUtils::len(l1k1));
	

	float A = CudaUtils::dot(l0k0, n0), B = CudaUtils::dot(l1k1, n1);
	float C = CudaUtils::dot(l0k0, n), D = CudaUtils::dot(l1k1, n);
	float E = CudaUtils::dot(l0k0, n1), F = CudaUtils::dot(l1k1, n0);

	float term3 = (2 * C + F) * 0.3333333f;
	float term4 = (2 * D + E) * 0.3333333f;

	//eeFlags[id] = ~((A < 2 * thickness && B < 2 * thickness && term3 < 2 * thickness && term4 < 2 * thickness) || (A >= 2 * thickness && B >= 2 * thickness && term3 >= 2 * thickness && term4 >= 2 * thickness));
	eeFlags[id] = ~((A < 0 && B < 0 && term3 < 0 && term4 < 0) || (A >= 0 && B >= 0 && term3 >= 0 && term4 >= 0));
}

__device__ int DeformableUtils::RaySphere(const float3 &rp1, const float3 &rp2, const float3 &sphereCenter, const float sphereRadius,
	float3 &qc)
{
	
	float3 dir = rp2 - rp1;

	if (CudaUtils::isZero(dir)) return 0;

	float len = CudaUtils::len(dir);

	dir /= len;

	float3 m = rp1 - sphereCenter;
	float b = CudaUtils::dot(m, dir);
	float c = CudaUtils::dot(m, m) - sphereRadius * sphereRadius;

	if (c > 0.f && b > 0.f)
		return false;

	float delta = b * b - c;

	if (delta < 0)
		return 0;

	float t = -b - sqrtf(delta);

	if (t < 0.f)
	{
		qc = rp1 + dir * t;
		return -1;
	}
		

	if (t > len)
		return 0;

	qc = rp1 + dir * t;
	return 1;
}

__device__ float DeformableUtils::RaySphere(const float3 & rp1, const float3 & rp2, const float3 & sphereCenter, const float sphereRadius)
{
	float3 dir = rp2 - rp1;

	if (CudaUtils::isZero(dir)) return -1.f;

	dir = CudaUtils::normalize(dir);

	float a = CudaUtils::dot(dir, dir);
	float3 rc = rp1 - sphereCenter;
	float b = 2.f * CudaUtils::dot(dir, rc);
	float c = CudaUtils::dot(rc, rc) - (sphereRadius * sphereRadius);

	float delta = b * b - 4 * a * c;
	if (delta < 0.f)
		return -1.f;

	return (-b - sqrtf(delta)) / (2.f * a);
}
