#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"


namespace CudaPrimitiveTests
{
	void DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, const thrust::device_vector<float3> &positions,
		thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, const float thickness);

	__device__ bool VFSanity(const int p, const int v1, const int v2, const int v3);

	__device__ bool EESanity(const int p1, const int p2, const int q1, const int q2);

	__global__ void DCDTriangleTests(const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ vfContacts, const uint64_t vfSize,
		Physics::PrimitiveContact * __restrict__ eeContacts, const uint64_t eeSize, const float thickness);

	__device__ void DCDTestVFs(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ vfContacts, const float thickness);

	__device__ void DCDTestVFs2(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ vfContacts, const float thickness);


	__device__ void DCDTestEEs(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ eeContacts, const float thickness);

	__device__ void DCDTestEEsBridson(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ eeContacts, const float thickness, const float eps);

	__device__ bool TestEdgeDegenerate(const float3 * __restrict__ positions, Physics::PrimitiveContact &contact, const float thickness);
}