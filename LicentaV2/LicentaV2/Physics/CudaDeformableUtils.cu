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
	int numBlocks = (verts.size() + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ResetVertColors << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > ((const int) verts.size(), thrust::raw_pointer_cast(&verts[0]));

	if (vfContactsSize + eeContactsSize == 0)
		return;

	numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	
	ColorCollidingFeatures << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (thrust::raw_pointer_cast(&verts[0]),
		thrust::raw_pointer_cast(&vfContacts[0]), vfContactsSize,
		thrust::raw_pointer_cast(&eeContacts[0]), eeContactsSize);
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

void DeformableUtils::CreateTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<Physics::AABBCollision> &rawAABBCols,
	const uint64_t timestamp, thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, void *&tempStorage, uint64_t &tempStorageSize)
{
	int filteredContactsSize = 0;

	CubWrap::SelectInPlace(rawAABBCols, EqualToValue(timestamp), filteredContactsSize, tempStorage, tempStorageSize);

	//printf("Cols before: %llu, after: %d\n", rawAABBCols.size(), filteredContactsSize);

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


	/*if (filteredContactsSize % 256 != 0)
		printf("CONTACTSSSS\n");*/


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

void DeformableUtils::DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle>& triangles, const thrust::device_vector<float3>& positions,
	thrust::device_vector<Physics::PrimitiveContact>& vfContacts, uint64_t & vfContactsSize,
	thrust::device_vector<Physics::PrimitiveContact>& eeContacts, uint64_t & eeContactsSize, const float thickness, void *&tempStorage, uint64_t &tempStorageSize)
{

	CudaPrimitiveTests::DCDTriangleTests(triangles, positions, vfContacts, vfContactsSize, eeContacts, eeContactsSize, thickness);

	//cudaDeviceSynchronize();
	//void SelectInPlace(thrust::device_vector<T> &inVec, const SelectorT selector, int &selectedCount, void *& tempStorage, uint64_t &tempStorageSize)
	PositiveCollisionTest selector;

	//printf("before vf: %llu, ee: %llu\n", vfContactsSize, eeContactsSize);
	int vfsz = 0, eesz = 0;
	//uint64_t requiredSize = 0;

	//int *dSelectedCount;
	//cudaMalloc(&dSelectedCount, sizeof(int));


	//thrust::device_vector<Physics::PrimitiveContact> testOut(vfContacts.size());
	//Physics::PrimitiveContact *rawOut = thrust::raw_pointer_cast(&testOut[0]);

	//Physics::PrimitiveContact *rawVec = thrust::raw_pointer_cast(&vfContacts[0]);

	//cub::DeviceSelect::If(NULL, requiredSize, rawVec, rawOut, dSelectedCount, vfContacts.size(), selector);

	//CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

	//cub::DeviceSelect::If(tempStorage, requiredSize, rawVec, rawOut, dSelectedCount, vfContacts.size(), selector);



	//cudaMemcpy(&vfsz, dSelectedCount, sizeof(int), cudaMemcpyDeviceToHost);

	//cudaFree(dSelectedCount);

	CubWrap::SelectInPlace(vfContacts, PositiveCollisionTest(), vfsz, tempStorage, tempStorageSize);
	CubWrap::SelectInPlace(eeContacts, PositiveCollisionTest(), eesz, tempStorage, tempStorageSize);

	vfContactsSize = (uint64_t)vfsz;
	eeContactsSize = (uint64_t)eesz;

	//printf("after vf: %llu, ee: %llu\n", vfContactsSize, eeContactsSize);

}

void DeformableUtils::CreateImpulses(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &velocities,
	const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
	const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize,
	thrust::device_vector<uint32_t> &impulseIDs, thrust::device_vector<float3> &impulseValues, uint64_t &impulsesSize,
	thrust::device_vector<uint32_t> &impulseRLEUniques, thrust::device_vector<int> &impulseRLECounts, int &impulseRunCount,
	thrust::device_vector<float3> &accumulatedImpulses, const float stiffness, const float vertexMass, const float timeStep, const float thickness,
	void *&tempStorage, uint64_t &tempStorageSize)
{

	cudaMemset(thrust::raw_pointer_cast(&accumulatedImpulses[0]), 0, accumulatedImpulses.size() * sizeof(float3));

	uint64_t totalImpulses = 4 * (vfContactsSize + eeContactsSize);

	if (impulsesSize < totalImpulses)
	{
		impulseIDs.resize(totalImpulses);
		impulseValues.resize(totalImpulses);
		impulseRLEUniques.resize(totalImpulses);
		impulseRLECounts.resize(totalImpulses);
		impulsesSize = totalImpulses;
	}


	if (totalImpulses == 0)
		return;


	int numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_CreateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&velocities[0]),
		thrust::raw_pointer_cast(&vfContacts[0]), vfContactsSize,
		thrust::raw_pointer_cast(&eeContacts[0]), eeContactsSize,
		thrust::raw_pointer_cast(&impulseIDs[0]),
		thrust::raw_pointer_cast(&impulseValues[0]), stiffness, vertexMass, timeStep, thickness);






	CubWrap::SortByKey(impulseIDs, impulseValues, totalImpulses, tempStorage, tempStorageSize);

	thrust::host_vector<uint32_t> hids(impulseIDs.begin(), impulseIDs.begin() + totalImpulses);
	thrust::host_vector<float3> hvals(impulseValues.begin(), impulseValues.begin() + totalImpulses);

	for (int i = 0; i < totalImpulses - 1; ++i)
	{
		if (hids[i] > hids[i + 1])
			printf("[%d] impulse: %d > %d -> (%f, %f, %f)\n", i, hids[i], hids[i + 1], hvals[i].x, hvals[i].y, hvals[i].z);
	}

	CubWrap::RLE(impulseIDs, totalImpulses, impulseRLEUniques, impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);

	thrust::host_vector<uint32_t> huniq(impulseRLEUniques.begin(), impulseRLEUniques.begin() + impulseRunCount);
	thrust::host_vector<int> hcount(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);

	//
	//for (int i = 0; i < impulseRunCount; ++i)
	//{
	//	printf("RLE: %d -> %d\n", huniq[i], hcount[i]);
	//}

	CubWrap::ExclusiveSumInPlace(impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);

	//thrust::host_vector<int> hPref(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);
	//for (int i = 0; i < impulseRunCount; ++i)
	//{
	//	printf("Prefix: %d - %d\n", hcount[i], hPref[i]);
	//}

	numBlocks = (impulseRunCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	if (impulseRunCount > accumulatedImpulses.size())
	{
		printf("EROAREOARE\n");
		
		for (int i = 0; i < impulseRunCount; ++i)
		{
			printf("[%d], RLE: %d -> %d\n", i, huniq[i], hcount[i]);
		}

		//for (int i = 0; i < totalImpulses - 1; ++i)
		//{
		//	if (hids[i] > hids[i + 1])
		//		printf("impulse: %d -> (%f, %f, %f)\n", hids[i], hvals[i].x, hvals[i].y, hvals[i].z);
		//}

		int z = 15;
	}
	AccumulateImpulses <<<numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (impulseRunCount, thrust::raw_pointer_cast(&impulseRLEUniques[0]),
		thrust::raw_pointer_cast(&impulseRLECounts[0]), thrust::raw_pointer_cast(&impulseValues[0]), totalImpulses, thrust::raw_pointer_cast(&accumulatedImpulses[0]));

	thrust::host_vector<float3> hacc(accumulatedImpulses);

	//for (int i = 0; i < hacc.size(); ++i)
	//{
	//	if (hacc[i].x == 0 && hacc[i].y == 0 && hacc[i].z == 0)
	//		continue;
	//	printf("[%d] final imp: (%f, %f, %f)\n", i, hacc[i].x, hacc[i].y, hacc[i].z);
	//}
}

void DeformableUtils::CreateImpulses(const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& velocities,
	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts, const uint64_t vfContactsSize,
	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts, const uint64_t eeContactsSize,
	Physics::DoubleBuffer<uint32_t>& impulseIDs, Physics::DoubleBuffer<float3>& impulseValues, uint64_t & impulsesSize,
	thrust::device_vector<uint32_t>& impulseRLEUniques, thrust::device_vector<int>& impulseRLECounts, int & impulseRunCount, thrust::device_vector<float3>& accumulatedImpulses,
	const float stiffness, const float vertexMass, const float timeStep, const float thickness, void *& tempStorage, uint64_t & tempStorageSize)
{
	cudaMemset(thrust::raw_pointer_cast(&accumulatedImpulses[0]), 0, accumulatedImpulses.size() * sizeof(float3));

	uint64_t totalImpulses = 4 * (vfContactsSize + eeContactsSize);

	if (impulsesSize < totalImpulses)
	{
		impulseIDs.buffers[0].resize(totalImpulses);
		impulseIDs.buffers[1].resize(totalImpulses);
		impulseValues.buffers[0].resize(totalImpulses);
		impulseValues.buffers[1].resize(totalImpulses);
		impulseRLEUniques.resize(totalImpulses);
		impulseRLECounts.resize(totalImpulses);
		impulsesSize = totalImpulses;
	}


	if (totalImpulses == 0)
		return;


	int numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_CreateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&velocities[0]),
		thrust::raw_pointer_cast(&vfContacts[0]), vfContactsSize,
		thrust::raw_pointer_cast(&eeContacts[0]), eeContactsSize,
		thrust::raw_pointer_cast(&impulseIDs.buffers[impulseIDs.selector][0]),
		thrust::raw_pointer_cast(&impulseValues.buffers[impulseIDs.selector][0]), stiffness, vertexMass, timeStep, thickness);



	CubWrap::SortByKey(impulseIDs, impulseValues, totalImpulses, tempStorage, tempStorageSize);


	//thrust::host_vector<uint32_t> hids(impulseIDs.buffers[impulseIDs.selector].begin(), impulseIDs.buffers[impulseIDs.selector].begin() + totalImpulses);
	//thrust::host_vector<float3> hvals(impulseValues.buffers[impulseValues.selector].begin(), impulseValues.buffers[impulseValues.selector].begin() + totalImpulses);

	//for (int i = 0; i < totalImpulses - 1; ++i)
	//{
	//	if (hids[i] > hids[i + 1])
	//		printf("[%d] impulse: %d > %d -> (%f, %f, %f)\n", i, hids[i], hids[i + 1], hvals[i].x, hvals[i].y, hvals[i].z);
	//}


	CubWrap::RLE(impulseIDs.buffers[impulseIDs.selector], totalImpulses, impulseRLEUniques, impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);

	//thrust::host_vector<uint32_t> huniq(impulseRLEUniques.begin(), impulseRLEUniques.begin() + impulseRunCount);
	//thrust::host_vector<int> hcount(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);

	//
	//for (int i = 0; i < impulseRunCount; ++i)
	//{
	//	printf("RLE: %d -> %d\n", huniq[i], hcount[i]);
	//}

	CubWrap::ExclusiveSumInPlace(impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);

	//thrust::host_vector<int> hPref(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);
	//for (int i = 0; i < impulseRunCount; ++i)
	//{
	//	printf("Prefix: %d - %d\n", hcount[i], hPref[i]);
	//}

	//numBlocks = (impulseRunCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	//if (impulseRunCount > accumulatedImpulses.size())
	//{
	//	printf("EROAREOARE\n");

	//	for (int i = 0; i < impulseRunCount; ++i)
	//	{
	//		printf("[%d], RLE: %d -> %d\n", i, huniq[i], hcount[i]);
	//	}

	//	//for (int i = 0; i < totalImpulses - 1; ++i)
	//	//{
	//	//	if (hids[i] > hids[i + 1])
	//	//		printf("impulse: %d -> (%f, %f, %f)\n", hids[i], hvals[i].x, hvals[i].y, hvals[i].z);
	//	//}

	//	int z = 15;
	//}
	AccumulateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (impulseRunCount, thrust::raw_pointer_cast(&impulseRLEUniques[0]),
		thrust::raw_pointer_cast(&impulseRLECounts[0]), thrust::raw_pointer_cast(&impulseValues.buffers[impulseValues.selector][0]), totalImpulses, thrust::raw_pointer_cast(&accumulatedImpulses[0]));
}

__global__ void DeformableUtils::_CreateImpulses(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
	const Physics::PrimitiveContact *__restrict__ vfs, const uint64_t vfSize,
	const Physics::PrimitiveContact *__restrict__ ees, const uint64_t eeSize,
	uint32_t *__restrict__ impulseIDs, float3 *__restrict__ impulseValues, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= vfSize + eeSize)
		return;

	if (id < vfSize)
	{
		_CreateImpulseVF(positions, velocities, vfs[id], impulseIDs, impulseValues, id * 4, stiffness, vertexMass, timeStep, thickness);
	}
	else
	{
		_CreateImpulseEE(positions, velocities, ees[id - vfSize], impulseIDs, impulseValues, id * 4, stiffness, vertexMass, timeStep, thickness);
	}
}

__device__ void DeformableUtils::_CreateImpulseVF(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
	const Physics::PrimitiveContact & contact,
	uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, const int myImpulseStart, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{
	
	float3 cv1 = contact.w1 * velocities[contact.v1];
	float3 cv2 = contact.w2 * velocities[contact.v2] + contact.w3 * velocities[contact.v3] + contact.w4 * velocities[contact.v4];
	float3 vr = cv1 - cv2;

	_CreateImpulse(positions, contact, vr, impulseIDs, impulseValues, myImpulseStart, stiffness, vertexMass, timeStep, thickness);

}

__device__ void DeformableUtils::_CreateImpulseEE(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
	const Physics::PrimitiveContact & contact,
	uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, const int myImpulseStart, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{

	float3 cv1 = contact.w1 * velocities[contact.v1] + contact.w2 * velocities[contact.v2];
	float3 cv2 = contact.w3 * velocities[contact.v3] + contact.w4 * velocities[contact.v4];
	float3 vr = cv1 - cv2;

	_CreateImpulse(positions, contact, vr, impulseIDs, impulseValues, myImpulseStart, stiffness, vertexMass, timeStep, thickness);
}

__device__ void DeformableUtils::_CreateImpulse(const float3 * __restrict__ positions, const Physics::PrimitiveContact & contact, const float3 & vr, uint32_t *__restrict__ impulseIDs,
	float3 *__restrict__ impulseValues, const int myImpulseStart, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{

	float d = thickness - CudaUtils::dot(contact.w1 * positions[contact.v1] + contact.w2 * positions[contact.v2] + contact.w3 * positions[contact.v3] + contact.w4 * positions[contact.v4], contact.n);
	float vn = CudaUtils::dot(vr, contact.n);

	float dts = (0.1f * d) / timeStep;
	float i = 0.f;

	if (vn < dts)
		i = -min(timeStep * stiffness * d, vertexMass * (dts - vn));

	if (vn < 0)
		i += vertexMass * vn * 0.5f;// *0.25f;


	i = CudaUtils::clamp(i, -vertexMass * 0.1f * thickness, vertexMass * 0.1f * thickness);

	float iPrime = (2 * i) / (contact.w1 * contact.w1 + contact.w2 * contact.w2 + contact.w3 * contact.w3 + contact.w4 * contact.w4);

	float3 iPrimeVec = iPrime * contact.n;

	impulseIDs[myImpulseStart] = contact.v1;
	impulseIDs[myImpulseStart + 1] = contact.v2;
	impulseIDs[myImpulseStart + 2] = contact.v3;
	impulseIDs[myImpulseStart + 3] = contact.v4;


	impulseValues[myImpulseStart] = contact.w1 * iPrimeVec;
	impulseValues[myImpulseStart + 1] = contact.w2 * iPrimeVec;
	impulseValues[myImpulseStart + 2] = contact.w3 * iPrimeVec;
	impulseValues[myImpulseStart + 3] = contact.w4 * iPrimeVec;
}

__global__ void DeformableUtils::AccumulateImpulses(const int impulseRunCount, const uint32_t *__restrict__ impulseRLEUniques, const int *__restrict__ impulseRLEPrefixSums,
	const float3 * __restrict__ impulseValues, const uint64_t impulseValuesSize, float3 *__restrict__ accumulatedImpulses)
{
	int id = CudaUtils::MyID();
	if (id >= impulseRunCount)
		return;

	int myAccImpulseID = impulseRLEUniques[id];

	int myStart = impulseRLEPrefixSums[id];
	int myEnd = id == impulseRunCount - 1 ? impulseValuesSize : impulseRLEPrefixSums[id + 1];

	int denom = myEnd - myStart;

	float3 acc = make_float3(0.f, 0.f, 0.f);

	for (int i = myStart; i < myEnd; ++i)
	{
		acc += impulseValues[i];
	}

	accumulatedImpulses[myAccImpulseID] = acc / denom;
}

void DeformableUtils::ApplyImpulses(thrust::device_vector<float3>& positions, const thrust::device_vector<float3> &prevPositions, 
	thrust::device_vector<float3>& velocities,
	const thrust::device_vector<float3>& impulses, const thrust::device_vector<bool>& fixedVerts, const float vertexMass, const float timeStep)
{
	const int numBlocks = (positions.size() + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ApplyImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > ((int)positions.size(), thrust::raw_pointer_cast(&positions[0]), 
		thrust::raw_pointer_cast(&prevPositions[0]),
		thrust::raw_pointer_cast(&velocities[0]), thrust::raw_pointer_cast(&impulses[0]), thrust::raw_pointer_cast(&fixedVerts[0]), vertexMass, timeStep);

}

__global__ void DeformableUtils::ApplyImpulses(const int particleCount, float3 *__restrict__ positions, const float3 *__restrict__ prevPositions,
	float3 *__restrict__ velocities,
	const float3 *__restrict__ impulses, const bool * __restrict__ fixedVerts, const float vertexMass, const float timeStep)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	if (fixedVerts[id])
		return;

	/*if (CudaUtils::len(impulses[id]) != 0.f)
		printf("[%d] accImp: (%f, %f, %f), \n", id, impulses[id].x / vertexMass, impulses[id].z / vertexMass, impulses[id].z / vertexMass);*/

	/*float3 newVel = impulses[id] / vertexMass;

	float3 velDir = newVel - velocities[id];

	velocities[id] += CudaUtils::normalize(velDir) * (min(0.1f * 0.25f / 20.f, CudaUtils::len(velDir)));*/

	velocities[id] += impulses[id] / vertexMass;
	positions[id] = prevPositions[id] + velocities[id] * timeStep;
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