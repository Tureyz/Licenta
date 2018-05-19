#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

namespace CudaMassSpring
{
	void ClothEngineStep(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions, thrust::device_vector<float3> &velocities,
		const thrust::device_vector<float> &masses, const thrust::device_vector<int> &aIDs, const thrust::device_vector<int> &bIDs, const thrust::device_vector<float> &ks,
		const thrust::device_vector<float> &l0s, const thrust::device_vector<int> &springInfo, const thrust::device_vector<int> &springIDs, const float3 gravity, const float dampingCoef,
		const float timeStep, const float bendcoef, const thrust::device_vector<bool> &fixedVerts);

	__global__ void ClothEngineStep(const int particleCount, float3 * __restrict__ positions, float3 *__restrict__ prevPositions, float3 *__restrict__ velocities,
		const float *__restrict__ masses, const int springCount, const int *__restrict__ aIDs, const int *__restrict__ bIDs, const float *__restrict__ ks, const float *__restrict__ l0s,
		const int *__restrict__ springInfo, const int *__restrict__ springIDs, const float3 gravity, const float dampingCoef, const float timeStep, const float bendCoef,
		const bool *__restrict__ fixedVerts);



	void AdjustSprings(thrust::device_vector<float3> &positions, const thrust::device_vector<int> &bIDs,
		const thrust::device_vector<float> &l0s, const float maxDeformation, const thrust::device_vector<int> &springInfo,
		const thrust::device_vector<int> &springIDs, const thrust::device_vector<bool> &fixedVerts,
		const thrust::device_vector<float>& ks, const float bendCoef);

	__global__ void _AdjustSprings(const int particleCount, float3 * __restrict__ positions,
		const int * __restrict__ bIDs, const float * __restrict__ l0s, const float maxDeformation, const int * __restrict__ springInfo, const int * __restrict__ springIDs,
		const bool * __restrict__ fixedVerts, const float * __restrict__ ks, const float bendCoef);

	__device__ float ComputeSpringDeformation(const float3 a, const float3 b, const float l0);
	
}