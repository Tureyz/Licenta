#include "CudaDeformableUtils.cuh"

#include <unordered_set>

#include "../Core/CudaUtils.cuh"

#ifndef __INTELLISENSE__
	#include "../Core/CubWrappers.cuh"
#endif 

#include "../Collision/CudaPrimitiveTests.cuh"


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

	const float3 posV1 = positions[triangles[id].m_v1];
	const float3 posV2 = positions[triangles[id].m_v2];
	const float3 posV3 = positions[triangles[id].m_v3];

	triangles[id].m_faceNormal = CudaUtils::normalize(CudaUtils::FaceNormal(posV1, posV2, posV3));


	//printf("[%d] faceNormal: (%f, %f, %f)\n", id, triangles[id].m_faceNormal.x, triangles[id].m_faceNormal.y, triangles[id].m_faceNormal.z);

	const float3 min = CudaUtils::min3(posV1, posV2, posV3) - thickness;
	const float3 max = CudaUtils::max3(posV1, posV2, posV3) + thickness;

	aabbMins[id + triangleCount - 1] = min;
	aabbMaxs[id + triangleCount - 1] = max;

	mortonCodes[id] = CudaUtils::Morton3D64(min + (max - min) / 2.f);

	//colSizes[id] = 0;
}

void DeformableUtils::UpdateTrianglesContinuous(const thrust::device_vector<float3>& positions,
	const thrust::device_vector<float3>& prevPositions,
	thrust::device_vector<Physics::CudaTriangle>& triangles,
	thrust::device_vector<uint64_t>& mortonCodes, thrust::device_vector<float3>& aabbMins, thrust::device_vector<float3>& aabbMaxs, const float thickness)
{
	const int triangleCount = triangles.size();

	int numBlocks = (triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_UpdateTrianglesContinuous << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(triangleCount, cu::raw(positions), cu::raw(prevPositions), cu::raw(triangles),
		cu::raw(mortonCodes), cu::raw(aabbMins), cu::raw(aabbMaxs), thickness);
}

__global__ void DeformableUtils::_UpdateTrianglesContinuous(const int triangleCount, const float3 *__restrict__ positions, const float3 *__restrict__ prevPositions, Physics::CudaTriangle *__restrict__ triangles, uint64_t *__restrict__ mortonCodes, float3 *__restrict__ aabbMins, float3 *__restrict__ aabbMaxs, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	const float3 posV1 = positions[triangles[id].m_v1];
	const float3 posV2 = positions[triangles[id].m_v2];
	const float3 posV3 = positions[triangles[id].m_v3];


	const float3 prevPosV1 = prevPositions[triangles[id].m_v1];
	const float3 prevPosV2 = prevPositions[triangles[id].m_v2];
	const float3 prevPosV3 = prevPositions[triangles[id].m_v3];

	triangles[id].m_faceNormal = CudaUtils::normalize(CudaUtils::FaceNormal(posV1, posV2, posV3));

	const float3 min = CudaUtils::min3(posV1, posV2, posV3) - thickness;
	const float3 max = CudaUtils::max3(posV1, posV2, posV3) + thickness;

	const float3 prevMin = CudaUtils::min3(prevPosV1, prevPosV2, prevPosV3) - thickness;
	const float3 prevMax = CudaUtils::max3(prevPosV1, prevPosV2, prevPosV3) + thickness;


	const float3 sweptMin = CudaUtils::minf3(min, prevMin);
	const float3 sweptMax = CudaUtils::maxf3(max, prevMax);
	aabbMins[id + triangleCount - 1] = sweptMin;
	aabbMaxs[id + triangleCount - 1] = sweptMax;

	mortonCodes[id] = CudaUtils::Morton3D64(sweptMin + (sweptMax - sweptMin) / 2.f);

}

void DeformableUtils::UpdateSweptAABBs(const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& prevPositions, const thrust::device_vector<Physics::CudaTriangle>& triangles, thrust::device_vector<float3>& aabbMins, thrust::device_vector<float3>& aabbMaxs, const float thickness)
{
	const int triangleCount = triangles.size();

	int numBlocks = (triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_UpdateSweptAABBs << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(triangleCount, cu::raw(positions), cu::raw(prevPositions),
		cu::raw(triangles),
		cu::raw(aabbMins), cu::raw(aabbMaxs), thickness);
}

__global__ void DeformableUtils::_UpdateSweptAABBs(const int triangleCount, const float3 *__restrict__ positions, const float3 *__restrict__ prevPositions,
	const Physics::CudaTriangle *__restrict__ triangles, float3 *__restrict__ aabbMins, float3 *__restrict__ aabbMaxs, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	const float3 posV1 = positions[triangles[id].m_v1];
	const float3 posV2 = positions[triangles[id].m_v2];
	const float3 posV3 = positions[triangles[id].m_v3];

	const float3 prevPosV1 = prevPositions[triangles[id].m_v1];
	const float3 prevPosV2 = prevPositions[triangles[id].m_v2];
	const float3 prevPosV3 = prevPositions[triangles[id].m_v3];

	const float3 min = CudaUtils::min3(posV1, posV2, posV3) - thickness;
	const float3 max = CudaUtils::max3(posV1, posV2, posV3) + thickness;

	const float3 prevMin = CudaUtils::min3(prevPosV1, prevPosV2, prevPosV3) - thickness;
	const float3 prevMax = CudaUtils::max3(prevPosV1, prevPosV2, prevPosV3) + thickness;


	aabbMins[id + triangleCount - 1] = CudaUtils::minf3(min, prevMin);
	aabbMaxs[id + triangleCount - 1] = CudaUtils::maxf3(max, prevMax);
}

void DeformableUtils::UpdateVertexNormals(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &rawVNs, thrust::device_vector<uint32_t> &rawVNIDs,
	thrust::device_vector<float3> &accVNs)
{
	const int triangleCount = triangles.size();

	int numBlocks = (triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	CreateVNs<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(triangleCount, cu::raw(triangles), cu::raw(rawVNs), cu::raw(rawVNIDs));


	const int rawVNCount = rawVNs.size();
	numBlocks = (rawVNCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	AccumulateVNs << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (rawVNCount, cu::raw(rawVNs), cu::raw(rawVNIDs), cu::raw(accVNs));

	const int particleCount = accVNs.size();
	numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	NormalizeVNs << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(accVNs));
}

__global__ void DeformableUtils::CreateVNs(const int triangleCount, const Physics::CudaTriangle *__restrict__ triangles, float3 *__restrict__ rawVNs, uint32_t *__restrict__ rawVNIDs)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;


	const Physics::CudaTriangle tri = triangles[id];

	const int myStart = id * 3;

	rawVNs[myStart] = tri.m_faceNormal;
	rawVNs[myStart + 1] = tri.m_faceNormal;
	rawVNs[myStart + 2] = tri.m_faceNormal;

	rawVNIDs[myStart] = tri.m_v1;
	rawVNIDs[myStart + 1] = tri.m_v2;
	rawVNIDs[myStart + 2] = tri.m_v3;
}

__global__ void DeformableUtils::AccumulateVNs(const int rawVNCount, const float3 *__restrict__ rawVNs, const uint32_t *__restrict__ rawVNIDs, float3 *__restrict__ accumulatedVNs)
{
	int id = CudaUtils::MyID();
	if (id >= rawVNCount)
		return;

	atomicAdd(&accumulatedVNs[rawVNIDs[id]].x, rawVNs[id].x);
	atomicAdd(&accumulatedVNs[rawVNIDs[id]].y, rawVNs[id].y);
	atomicAdd(&accumulatedVNs[rawVNIDs[id]].z, rawVNs[id].z);
}

__global__ void DeformableUtils::NormalizeVNs(const int particleCount, float3 *__restrict__ accVNs)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	accVNs[id] = CudaUtils::normalize(accVNs[id]);
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

void DeformableUtils::FinalVerticesUpdate(thrust::device_vector<Rendering::VertexFormat>& verts, const thrust::device_vector<Physics::CudaTriangle>& triangles,
	const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& vertexNormals)
{
	const int particleCount = verts.size();

	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_FinalVerticesUpdate << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(verts), cu::raw(triangles), cu::raw(positions), cu::raw(vertexNormals));
}

__global__ void DeformableUtils::_FinalVerticesUpdate(const int particleCount, Rendering::VertexFormat *__restrict__ verts, const Physics::CudaTriangle *__restrict__ triangles, 
	const float3 *__restrict__ positions, const float3 *__restrict__ vertexNormals)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	float3 pos = positions[id];
	float3 n = vertexNormals[id];

	verts[id].m_position.x = pos.x;
	verts[id].m_position.y = pos.y;
	verts[id].m_position.z = pos.z;

	verts[id].m_normal.x = n.x;
	verts[id].m_normal.y = n.y;
	verts[id].m_normal.z = n.z;
}

void DeformableUtils::SortMortons(thrust::device_vector<uint64_t>& mortonCodes, thrust::device_vector<Physics::CudaTriangle>& triangles, void *& tempStorage, uint64_t &tempStorageSize)
{
	CubWrap::SortByKey(mortonCodes, triangles, (int) mortonCodes.size(), tempStorage, tempStorageSize);
}

void DeformableUtils::ColorCollidingFeatures(thrust::device_vector<Rendering::VertexFormat>& verts,
	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts, const uint64_t vfContactsSize,
	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts, const uint64_t eeContactsSize)
{
	if (vfContactsSize + eeContactsSize == 0)
		return;

	const int numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	
	ColorCollidingFeatures << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (thrust::raw_pointer_cast(&verts[0]),
		thrust::raw_pointer_cast(&vfContacts[0]), vfContactsSize,
		thrust::raw_pointer_cast(&eeContacts[0]), eeContactsSize);
}

void DeformableUtils::ResetVertColors(thrust::device_vector<Rendering::VertexFormat>& verts)
{
	const int vertsSize = (int)verts.size();
	const int numBlocks = (vertsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ResetVertColors<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(vertsSize, cu::raw(verts));
}

__global__ void DeformableUtils::ResetVertColors(const int vertsSize, Rendering::VertexFormat * __restrict__ verts)
{
	int id = CudaUtils::MyID();
	if (id >= vertsSize)
		return;

	verts[id].m_color.r = 0.3f;
	verts[id].m_color.g = 0.3f;
	verts[id].m_color.b = 0.3f;
}

__global__ void DeformableUtils::ColorCollidingFeatures(Rendering::VertexFormat *__restrict__ verts,
	const Physics::PrimitiveContact * __restrict__ vfContacts, const uint64_t vfContactsSize,
	const Physics::PrimitiveContact * __restrict__ eeContacts, const uint64_t eeContactsSize)
{
	int id = CudaUtils::MyID();
	if (id >= vfContactsSize + eeContactsSize)
		return;

	if (id < vfContactsSize)
	{
		verts[vfContacts[id].v1].m_color.r = 0.5f;
		verts[vfContacts[id].v2].m_color.r = 0.5f;
		verts[vfContacts[id].v3].m_color.r = 0.5f;
		verts[vfContacts[id].v4].m_color.r = 0.5f;

		verts[vfContacts[id].v1].m_color.b = 0.f;
		verts[vfContacts[id].v2].m_color.b = 0.f;
		verts[vfContacts[id].v3].m_color.b = 0.f;
		verts[vfContacts[id].v4].m_color.b = 0.f;
	}
	else
	{
		verts[eeContacts[id - vfContactsSize].v1].m_color.g = 0.5f;
		verts[eeContacts[id - vfContactsSize].v2].m_color.g = 0.5f;
		verts[eeContacts[id - vfContactsSize].v3].m_color.g = 0.5f;
		verts[eeContacts[id - vfContactsSize].v4].m_color.g = 0.5f;

		verts[eeContacts[id - vfContactsSize].v1].m_color.b = 0.f;
		verts[eeContacts[id - vfContactsSize].v2].m_color.b = 0.f;
		verts[eeContacts[id - vfContactsSize].v3].m_color.b = 0.f;
		verts[eeContacts[id - vfContactsSize].v4].m_color.b = 0.f;
	}
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