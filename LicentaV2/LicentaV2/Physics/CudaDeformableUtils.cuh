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

	struct EqualToValue
	{
		uint64_t value;

		__host__ __device__ __forceinline__ EqualToValue(uint64_t val) : value(val) {}
		__host__ __device__ __forceinline__ bool operator()(const Physics::AABBCollision &a) const { return a.m_timestamp == value; }
	};



	bool CheckMortonDuplicates(const thrust::device_vector<unsigned int> mortonCodes);

	void CreateTriangles(const thrust::device_vector<float3> &positions, const thrust::device_vector<unsigned int> &indices, thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);

	__global__ void _CreateTriangles(const int triangleCount, const float3 * __restrict__ positions, const unsigned int * __restrict__ indices, Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);


	void UpdateTriangles(const thrust::device_vector<float3> &positions, thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);//, thrust::device_vector<int> &colSizes);

	__global__ void _UpdateTriangles(const int triangleCount, const float3 * __restrict__ positions, Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);


	void WorldConstraints(thrust::device_vector<float3> &positions);

	__global__ void _WorldConstraints(const int particleCount, float3 * __restrict__ positions, const float3 worldMin, const float3 worldMax);

	void AddWind(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &positions, const thrust::device_vector<float> &masses,
		const float timeStep);

	__global__ void _AddWind(const int triangleCount, const Physics::CudaTriangle * __restrict__ triangles, float3 * __restrict__ positions, const float * __restrict__ masses,
		const float3 windDirection, const float timeStep);


	void FinalVerticesUpdate(thrust::device_vector<Rendering::VertexFormat> &verts, const thrust::device_vector<Physics::CudaTriangle> &triangles, const thrust::device_vector<float3> &positions);

	__global__ void _FinalVerticesUpdate(const int particleCount, Rendering::VertexFormat * __restrict__ verts, const Physics::CudaTriangle * __restrict__ triangles,
		const float3 * __restrict__ positions);


	void SortMortons(thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<Physics::CudaTriangle> &triangles, void *& tempStorage, uint64_t &tempStorageSize);


	void CreateTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<Physics::AABBCollision> &rawAABBCols,		
		const uint64_t timestamp, thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, void *&tempStorage, uint64_t &tempStorageSize);

	__global__ void CreateTriangleTests(const int filteredContactsSize, const Physics::CudaTriangle * __restrict__ triangles, const Physics::AABBCollision * __restrict__ filteredCols,
		Physics::PrimitiveContact * __restrict__ vfContacts, Physics::PrimitiveContact * __restrict__ eeContacts);

	//void FlattenAABBCollisionList(thrust::device_vector<Physics::AABBCollision*> &collisions, thrust::device_vector<int> &collisionSizes,
	//	thrust::device_vector<Physics::AABBCollision> &output, void *&tempStorage, uint64_t &tempStorageSize);

	//__global__ void _FlattenAABBCollisionList(const int threadCount, Physics::AABBCollision ** __restrict__ collisions, int * __restrict__ collisionSizes,
	//	Physics::AABBCollision * __restrict__ output);
}