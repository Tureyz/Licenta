#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"


namespace DeformableUtils
{
	/*struct EqualToValue
	{
		uint64_t value;

		__host__ __device__ __forceinline__ EqualToValue(uint64_t val) : value(val) {}
		__host__ __device__ __forceinline__ bool operator()(const Physics::AABBCollision &a) const { return a.m_timestamp == value; }
	};*/

	//struct PositiveCollisionTest
	//{
	//	__host__ __device__ __forceinline__ bool operator()(const Physics::PrimitiveContact &a) const { return a.contact; }
	//};

	void CreateTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<Physics::AABBCollision> &rawAABBCols, const thrust::device_vector<bool> &flaggedAABBCols,
		thrust::device_vector<Physics::PrimitiveContact> &vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t &vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact> &eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t &eeContactsSize,
		void *&tempStorage, uint64_t &tempStorageSize);

	__global__ void CreateTriangleTests(const int filteredContactsSize, const Physics::CudaTriangle * __restrict__ triangles, const Physics::AABBCollision * __restrict__ filteredCols,
		Physics::PrimitiveContact * __restrict__ vfContacts, Physics::PrimitiveContact * __restrict__ eeContacts);

	void DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle>& triangles, const thrust::device_vector<float3>& positions,
		thrust::device_vector<Physics::PrimitiveContact>& vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t & vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact>& eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t & eeContactsSize,
		const float thickness, void *&tempStorage, uint64_t &tempStorageSize);

	void CCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle>& triangles,
		const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& velocities,
		thrust::device_vector<Physics::PrimitiveContact>& vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t & vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact>& eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t & eeContactsSize,
		const float thickness, const float timeStep, void *& tempStorage, uint64_t & tempStorageSize);

}