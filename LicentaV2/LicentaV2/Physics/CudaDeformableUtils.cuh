#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "Structs.h"
#include "../Rendering/VertexFormat.h"



namespace DeformableUtils
{
	const float3 WORLD_MIN = make_float3(0, 0, 0);
	const float3 WORLD_MAX = make_float3(1, 1, 1);
	const float3 WIND_DIR = make_float3(0, 0, 0.2f);


	bool CheckMortonDuplicates(const thrust::device_vector<unsigned int> mortonCodes);

	void CreateTriangles(const thrust::device_vector<float3> &positions, const thrust::device_vector<unsigned int> &indices, thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);

	__global__ void _CreateTriangles(const int triangleCount, const float3 * __restrict__ positions, const unsigned int * __restrict__ indices, Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);


	void UpdateTriangles(const thrust::device_vector<float3> &positions, thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);//, thrust::device_vector<int> &colSizes);

	__global__ void _UpdateTriangles(const int triangleCount, const float3 * __restrict__ positions, Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);

	void UpdateTrianglesContinuous(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &prevPositions,
		thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);

	__global__ void _UpdateTrianglesContinuous(const int triangleCount, const float3 * __restrict__ positions, const float3 * __restrict__ prevPositions,
		Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);

	void UpdateSweptAABBs(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &prevPositions,
		const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);

	__global__ void _UpdateSweptAABBs(const int triangleCount, const float3 * __restrict__ positions, const float3 * __restrict__ prevPositions,
		const Physics::CudaTriangle * __restrict__ triangles, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);

	void UpdateVertexNormals(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &rawVNs, thrust::device_vector<uint32_t> &rawVNIDs,
		thrust::device_vector<float3> &accVNs);

	__global__ void CreateVNs(const int triangleCount, const Physics::CudaTriangle *__restrict__ triangles, float3 *__restrict__ rawVNs, uint32_t *__restrict__ rawVNIDs);

	__global__ void AccumulateVNs(const int rawVNCount, const float3 *__restrict__ rawVNs, const uint32_t *__restrict__ rawVNIDs, float3 *__restrict__ accumulatedVNs);

	__global__ void NormalizeVNs(const int particleCount, float3 * __restrict__ accVNs);

	void WorldConstraints(thrust::device_vector<float3> &positions);

	__global__ void _WorldConstraints(const int particleCount, float3 * __restrict__ positions, const float3 worldMin, const float3 worldMax);

	void AddWind(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &positions, const thrust::device_vector<float> &masses,
		const float timeStep);

	__global__ void _AddWind(const int triangleCount, const Physics::CudaTriangle * __restrict__ triangles, float3 * __restrict__ positions, const float * __restrict__ masses,
		const float3 windDirection, const float timeStep);


	void FinalVerticesUpdate(thrust::device_vector<Rendering::VertexFormat> &verts, const thrust::device_vector<Physics::CudaTriangle> &triangles,
		const thrust::device_vector<float3> &positions, const thrust::device_vector<float3>& vertexNormals);

	__global__ void _FinalVerticesUpdate(const int particleCount, Rendering::VertexFormat * __restrict__ verts, const Physics::CudaTriangle * __restrict__ triangles,
		const float3 * __restrict__ positions, const float3 * __restrict__ vertexNormals);


	void SortMortons(thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<Physics::CudaTriangle> &triangles, void *& tempStorage, uint64_t &tempStorageSize);


	void ColorCollidingFeatures(thrust::device_vector<Rendering::VertexFormat> &verts,
		const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
		const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize);

	void ResetVertColors(thrust::device_vector<Rendering::VertexFormat> &verts);

	__global__ void ResetVertColors(const int vertsSize, Rendering::VertexFormat * __restrict__ verts);

	__global__ void ColorCollidingFeatures(Rendering::VertexFormat * __restrict__ verts,
		const Physics::PrimitiveContact * __restrict__ vfContacts, const uint64_t vfContactsSize,
		const Physics::PrimitiveContact * __restrict__ eeContacts, const uint64_t eeContactsSize);

	


	


	//void FlattenAABBCollisionList(thrust::device_vector<Physics::AABBCollision*> &collisions, thrust::device_vector<int> &collisionSizes,
	//	thrust::device_vector<Physics::AABBCollision> &output, void *&tempStorage, uint64_t &tempStorageSize);

	//__global__ void _FlattenAABBCollisionList(const int threadCount, Physics::AABBCollision ** __restrict__ collisions, int * __restrict__ collisionSizes,
	//	Physics::AABBCollision * __restrict__ output);
}