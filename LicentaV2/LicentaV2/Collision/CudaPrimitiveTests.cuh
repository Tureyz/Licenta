#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"

namespace CudaPrimitiveTests
{
	void DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, const thrust::device_vector<float3> &positions,
		thrust::device_vector<Physics::PrimitiveContact> &vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t &vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact> &eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t &eeContactsSize,
		const float thickness);

	__global__ void DCDTriangleTests(const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ vfContacts, bool * __restrict__ vfFlags, const uint64_t vfSize,
		Physics::PrimitiveContact * __restrict__ eeContacts, bool * __restrict__ eeFlags, const uint64_t eeSize, const float thickness);

	__device__ void DCDTestVFs(const int id, const Physics::CudaTriangle * __restrict__ triangles,
		const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ vfContacts, bool * __restrict__ vfFlags, const float thickness);


	__device__ void DCDTestEEsBridson(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
		Physics::PrimitiveContact * __restrict__ eeContacts, bool * __restrict__ eeFlags, const float thickness, const float eps);



	void CCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle>& triangles,
		const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& velocities,
		thrust::device_vector<Physics::PrimitiveContact>& vfContacts, thrust::device_vector<bool> &vfFlags, uint64_t & vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact>& eeContacts, thrust::device_vector<bool> &eeFlags, uint64_t & eeContactsSize,
		const float thickness, const float timeStep);

	__global__ void CCDTriangleTests(const Physics::CudaTriangle *__restrict__ triangles,
		const float3 *__restrict__ positions, const float3 *__restrict__ velocities,
		Physics::PrimitiveContact *__restrict__ vfContacts, bool * __restrict__ vfFlags, const uint64_t vfSize,
		Physics::PrimitiveContact *__restrict__ eeContacts, bool * __restrict__ eeFlags, const uint64_t eeSize,
		const float thickness, const float timeStep);


	__device__ void CCDTestVFs(const int id, const Physics::CudaTriangle *__restrict__ triangles,
		const float3 *__restrict__ positions, const float3 *__restrict__ velocities,
		Physics::PrimitiveContact *__restrict__ vfContacts, bool * __restrict__ vfFlags, const float thickness,
		const float timeStep);

	__device__ void CCDTestEEsBridson(const int id, const Physics::CudaTriangle *__restrict__ triangles,
		const float3 *__restrict__ positions, const float3 *__restrict__ velocities,
		Physics::PrimitiveContact *__restrict__ eeContacts, bool * __restrict__ eeFlags, const float thickness,
		const float eps, const float timeStep);

	__device__ bool TestEE(const float3 &x1, const float3 &x2, const float3 &x3, const float3 &x4, Physics::PrimitiveContact &contact,
		const float thickness, const float eps);

	__device__ bool TestVF(const float3 &x4, const float3 &x1, const float3 &x2, const float3 &x3, Physics::PrimitiveContact &contact,
		const float thickness);

	__device__ bool VFSanity(const int p, const int v1, const int v2, const int v3);

	__device__ bool EESanity(const int p1, const int p2, const int q1, const int q2);	

	__device__ bool TestEdgeDegenerate(const float3 &x1, const float3 &x3, const float3 &x21, const float3 &x43,
		Physics::PrimitiveContact &contact, const float thickness);



	__global__ void FeatureAABBTests(const int particleCount, const float3 * __restrict__  particleMins, const float3 * __restrict__ particleMaxs,
		const int * __restrict__ edgeMap, const int * __restrict__ edgev1s, const int * __restrict__ edgev2s,
		const float3 * __restrict__ edgeMins, const float3 * __restrict__ edgeMaxs,
		const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ triangleMins,
		const float3 * __restrict__ triangleMaxs,
		const Physics::PrimitiveContact *__restrict__ vfContacts, bool * __restrict__ vfFlags, const uint64_t vfSize,
		const Physics::PrimitiveContact *__restrict__ eeContacts, bool * __restrict__ eeFlags, const uint64_t eeSize);

	__device__ void VFAABBTest(const int id, const float3 * __restrict__  particleMins, const float3 * __restrict__ particleMaxs,
		const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ triangleMins, const float3 * __restrict__ triangleMaxs,
		const Physics::PrimitiveContact *__restrict__ vfContacts, bool * __restrict__ vfFlags, const uint64_t vfSize);

	__device__ void EEAABBTest(const int id, const int particleCount, const int * __restrict__ edgeMap,
		const int * __restrict__ edgev1s, const int * __restrict__ edgev2s,
		const float3 * __restrict__ edgeMins, const float3 * __restrict__ edgeMaxs,
		const Physics::PrimitiveContact *__restrict__ eeContacts, bool * __restrict__ eeFlags, const uint64_t eeSize);


	//__device__ void DCDTestVFs2(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
	//	Physics::PrimitiveContact * __restrict__ vfContacts, const float thickness);

	//__device__ void DCDTestEEs(const int id, const Physics::CudaTriangle * __restrict__ triangles, const float3 * __restrict__ positions,
	//	Physics::PrimitiveContact * __restrict__ eeContacts, const float thickness);
}