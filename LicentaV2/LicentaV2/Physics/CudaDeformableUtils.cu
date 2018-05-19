#include "CudaDeformableUtils.cuh"

#include <unordered_set>

#include "../Core/CudaUtils.cuh"
#include "../Core/CubWrappers.cuh"



bool DeformableUtils::CheckMortonDuplicates(const thrust::device_vector<unsigned int> mortonCodes)
{
	const thrust::host_vector<unsigned int> hMortons = mortonCodes;


	std::unordered_set<unsigned int> s(hMortons.begin(), hMortons.end());

	return s.size() < hMortons.size();
}

void DeformableUtils::CreateTriangles(const thrust::device_vector<float3> &positions, const thrust::device_vector<unsigned int> &indices, thrust::device_vector<Physics::CudaTriangle> &triangles,
	thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness)
{
	const int triangleCount = indices.size() / 3;

	int numBlocks = (triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_CreateTriangles<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(triangleCount, thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&indices[0]),
		thrust::raw_pointer_cast(&triangles[0]),
		thrust::raw_pointer_cast(&mortonCodes[0]), thrust::raw_pointer_cast(&aabbMins[0]), thrust::raw_pointer_cast(&aabbMaxs[0]), thickness);

}

__global__ void DeformableUtils::_CreateTriangles(const int triangleCount, const float3 * __restrict__ positions, const unsigned int * __restrict__ indices, Physics::CudaTriangle * __restrict__ triangles,
	uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	int startID = id * 3;

	triangles[id].m_v1 = indices[startID];
	triangles[id].m_v2 = indices[startID + 1];
	triangles[id].m_v3 = indices[startID + 2];

	float3 posV1 = positions[triangles[id].m_v1];
	float3 posV2 = positions[triangles[id].m_v2];
	float3 posV3 = positions[triangles[id].m_v3];

	triangles[id].m_faceNormal = CudaUtils::FaceNormal(posV1, posV2, posV3);


	float3 min = CudaUtils::min3(posV1, posV2, posV3);
	float3 max = CudaUtils::max3(posV1, posV2, posV3);

	aabbMins[id] = min;
	aabbMaxs[id] = max;

	mortonCodes[id] = CudaUtils::Morton3D64(min + (max - min) / 2.f);
}

void DeformableUtils::UpdateTriangles(const thrust::device_vector<float3> &positions, thrust::device_vector<Physics::CudaTriangle> &triangles,
	thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness)//, thrust::device_vector<int> &colSizes)
{
	const int triangleCount = triangles.size();

	int numBlocks = (triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_UpdateTriangles<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(triangleCount, thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&triangles[0]),
		thrust::raw_pointer_cast(&mortonCodes[0]), thrust::raw_pointer_cast(&aabbMins[0]), thrust::raw_pointer_cast(&aabbMaxs[0]), thickness);// , thrust::raw_pointer_cast(&colSizes[0]));
}


__global__ void DeformableUtils::_UpdateTriangles(const int triangleCount, const float3 * __restrict__ positions, Physics::CudaTriangle * __restrict__ triangles,
	uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

//	printf("[%d] v1: %d, v2: %d, v3: %d\n", id, triangles[id].m_v1, triangles[id].m_v2, triangles[id].m_v3);

	float3 posV1 = positions[triangles[id].m_v1];
	float3 posV2 = positions[triangles[id].m_v2];
	float3 posV3 = positions[triangles[id].m_v3];

	triangles[id].m_faceNormal = CudaUtils::normalize(CudaUtils::FaceNormal(posV1, posV2, posV3));


	//printf("[%d] faceNormal: (%f, %f, %f)\n", id, triangles[id].m_faceNormal.x, triangles[id].m_faceNormal.y, triangles[id].m_faceNormal.z);

	float3 min = CudaUtils::min3(posV1, posV2, posV3) - thickness;
	float3 max = CudaUtils::max3(posV1, posV2, posV3) + thickness;

	aabbMins[id + triangleCount - 1] = min;
	aabbMaxs[id + triangleCount - 1] = max;

	mortonCodes[id] = CudaUtils::Morton3D64(min + (max - min) / 2.f);

	//colSizes[id] = 0;
}

void DeformableUtils::WorldConstraints(thrust::device_vector<float3>& positions)
{
	const int particleCount = positions.size();

	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_WorldConstraints<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(particleCount, thrust::raw_pointer_cast(&positions[0]), WORLD_MIN, WORLD_MAX);
}

__global__ void DeformableUtils::_WorldConstraints(const int particleCount, float3 *__restrict__ positions, const float3 worldMin, const float3 worldMax)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	positions[id].x = min(max(positions[id].x, worldMin.x), worldMax.x);
	positions[id].y = min(max(positions[id].y, worldMin.y), worldMax.y);
	positions[id].z = min(max(positions[id].z, worldMin.z), worldMax.z);
}

void DeformableUtils::AddWind(const thrust::device_vector<Physics::CudaTriangle>& triangles, thrust::device_vector<float3>& positions, const thrust::device_vector<float> &masses,
	const float timeStep)
{
	const int triangleCount = triangles.size();

	int numBlocks = (triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_AddWind <<<numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (triangleCount, thrust::raw_pointer_cast(&triangles[0]), thrust::raw_pointer_cast(&positions[0]),
		thrust::raw_pointer_cast(&masses[0]), WIND_DIR, timeStep);

}

__global__ void DeformableUtils::_AddWind(const int triangleCount, const Physics::CudaTriangle *__restrict__ triangles, float3 *__restrict__ positions, 
	const float * __restrict__ masses, const float3 windDirection, const float timeStep)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	float3 force = triangles[id].m_faceNormal * CudaUtils::dot(triangles[id].m_faceNormal, windDirection);

	float3 a1 = force / masses[triangles[id].m_v1];
	float3 a2 = force / masses[triangles[id].m_v2];
	float3 a3 = force / masses[triangles[id].m_v3];

	float ts2 = timeStep * timeStep;

	a1 *= ts2;
	a2 *= ts2;
	a3 *= ts2;


	atomicAdd(&positions[triangles[id].m_v1].x, a1.x);
	atomicAdd(&positions[triangles[id].m_v1].y, a1.y);
	atomicAdd(&positions[triangles[id].m_v1].z, a1.z);

	atomicAdd(&positions[triangles[id].m_v2].x, a2.x);
	atomicAdd(&positions[triangles[id].m_v2].y, a2.y);
	atomicAdd(&positions[triangles[id].m_v2].z, a2.z);

	atomicAdd(&positions[triangles[id].m_v3].x, a3.x);
	atomicAdd(&positions[triangles[id].m_v3].y, a3.y);
	atomicAdd(&positions[triangles[id].m_v3].z, a3.z);
	
}

void DeformableUtils::FinalVerticesUpdate(thrust::device_vector<Rendering::VertexFormat>& verts, const thrust::device_vector<Physics::CudaTriangle>& triangles, const thrust::device_vector<float3>& positions)
{
	const int particleCount = verts.size();

	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_FinalVerticesUpdate << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, thrust::raw_pointer_cast(&verts[0]),
		thrust::raw_pointer_cast(&triangles[0]), thrust::raw_pointer_cast(&positions[0]));
}

__global__ void DeformableUtils::_FinalVerticesUpdate(const int particleCount, Rendering::VertexFormat *__restrict__ verts, const Physics::CudaTriangle *__restrict__ triangles, const float3 *__restrict__ positions)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	float3 pos = positions[id];
	
	verts[id].m_position.x = pos.x;
	verts[id].m_position.y = pos.y;
	verts[id].m_position.z = pos.z;
}

void DeformableUtils::SortMortons(thrust::device_vector<uint64_t>& mortonCodes, thrust::device_vector<Physics::CudaTriangle>& triangles, void *& tempStorage, uint64_t &tempStorageSize)
{
	CubWrap::SortByKey(mortonCodes, triangles, tempStorage, tempStorageSize);
}

void DeformableUtils::CreateTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<Physics::AABBCollision> &rawAABBCols,
	const uint64_t timestamp, thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, void *&tempStorage, uint64_t &tempStorageSize)
{
	int filteredContactsSize = 0;

	CubWrap::SelectInPlace(rawAABBCols, EqualToValue(timestamp), filteredContactsSize, tempStorage, tempStorageSize);

	printf("Cols before: %llu, after: %d\n", rawAABBCols.size(), filteredContactsSize);

	vfContactsSize = 6 * filteredContactsSize;
	eeContactsSize = 9 * filteredContactsSize;

	if (vfContacts.size() < vfContactsSize)
	{
		vfContacts.resize(vfContactsSize);
	}

	if (eeContacts.size() < eeContactsSize)
	{
		eeContacts.resize(eeContactsSize);
	}

	if (filteredContactsSize == 0)
		return;

	if (filteredContactsSize % 256 != 0)
		printf("CONTACTSSSS\n");

	const int numBlocks = (filteredContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	CreateTriangleTests<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(filteredContactsSize, thrust::raw_pointer_cast(&triangles[0]), thrust::raw_pointer_cast(&rawAABBCols[0]),
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
























//void DeformableUtils::FlattenAABBCollisionList(thrust::device_vector<Physics::AABBCollision*>& collisions, thrust::device_vector<int> & collisionSizes,
//	thrust::device_vector<Physics::AABBCollision>& output, void *& tempStorage, uint64_t &tempStorageSize)
//{
//	uint64_t requiredSize = 0;
//
//	int *rawCollisionSizes = thrust::raw_pointer_cast(&collisionSizes[1]);
//
//	cub::DeviceScan::InclusiveSum(NULL, requiredSize, rawCollisionSizes, rawCollisionSizes, collisionSizes.size() - 1);
//
//
//	if (requiredSize > tempStorageSize)
//	{
//		if (tempStorage != NULL)
//		{
//			cudaFree(tempStorage);
//		}
//
//		cudaMalloc(&tempStorage, requiredSize);
//
//		tempStorageSize = requiredSize;
//	}
//
//	cub::DeviceScan::InclusiveSum(tempStorage, requiredSize, rawCollisionSizes, rawCollisionSizes, collisionSizes.size() - 1);
//
//	//int sz = 0;
//	//cudaMemcpy(&sz, rawCollisionSizes + collisionSizes.size() - 2, sizeof(int), cudaMemcpyDeviceToHost);
//
//	//if (sz == 0)
//	//	return;
//
//	/*output.resize(sz);
//
//
//	const int threadCount = collisions.size();
//
//	int numBlocks = (threadCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;
//
//	_FlattenAABBCollisionList<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(threadCount, thrust::raw_pointer_cast(&collisions[0]), thrust::raw_pointer_cast(&collisionSizes[0]),
//	thrust::raw_pointer_cast(&output[0]));*/
//}
//
//__global__ void DeformableUtils::_FlattenAABBCollisionList(const int threadCount, Physics::AABBCollision ** __restrict__ collisions, int * __restrict__ collisionSizes,
//	Physics::AABBCollision * __restrict__ output)
//{
//	int id = CudaUtils::MyID();
//	if (id >= threadCount)
//		return;
//
//
//	int j = 0;
//	for (int i = collisionSizes[id]; i < collisionSizes[id + 1]; ++i)
//	{
//		output[i] = collisions[id][j++];
//	}
//}