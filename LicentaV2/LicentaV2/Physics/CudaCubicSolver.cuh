#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"

namespace Physics
{
	__device__ const float EPS = 1.f / 1000000;

	__device__ void FindCoplanarityTimes(const float3 &x1, const float3 &x2, const float3 &x3, const float3 &x4,
		const float3 &v1, const float3 &v2, const float3 &v3, const float3 &v4, float *times, int &timesCount);

	__device__ void FindCriticalPoints(const float3 &x21, const float3 &x31, const float3 &x41,
		const float3 &v21, const float3 &v31, const float3 &v41, float &p1, float &p2, int &pointsCount);

	__device__ float ComputeTripleScalar(const float3 &x21, const float3 &x31, const float3 &x41,
		const float3 &v21, const float3 &v31, const float3 &v41, const float t);

	__device__ float Secant(const float3 &x21, const float3 &x31, const float3 &x41,
		const float3 &v21, const float3 &v31, const float3 &v41, const float a, const float b, const int maxIterations);
}