#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

namespace CudaPBD
{
	void DampVelocities(const thrust::device_vector<float3> &positions, const thrust::device_vector<float> &masses,
		const float dampingCoef, thrust::device_vector<float3> &velocities, thrust::device_vector<float3> &aux1,
		thrust::device_vector<float3> &aux2,
		void *&tempStorage, uint64_t tempStorageSize);

	
	__global__ void ComputeXMVMs(const int particleCount, const float3 * __restrict__ pos, const float3 * __restrict__ vel,
		const float * __restrict__ mass,
		float3 * __restrict__ XMs, float3 * __restrict__ VMs);


	__global__ void ComputeL(const int particleCount, const float3 *__restrict__ pos, const float3 xcm, float3 *vimi, float3 *out);

	__global__ void ComputeIs(const int particleCount, const float3 *__restrict__ pos, const float3 xcm,
		const float * __restrict__ mass, float3 * __restrict__ aux1, float3 * __restrict__ aux2);

	__global__ void UpdateVels(const int particleCount, const float3 * __restrict__ pos,
		const float3 omega, const float3 xcm, const float3 vcm, const float dampingCoef, float3 * __restrict__ vel);
}