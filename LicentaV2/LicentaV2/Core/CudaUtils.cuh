#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
//#include <math_functions.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "../Dependencies/glm/glm.hpp"

#define cudaCheckError() {                                          \
 cudaError_t e=cudaGetLastError();                                 \
 if(e!=cudaSuccess) {                                              \
   printf("Cuda failure %s:%d: '%s'\n",__FILE__,__LINE__,cudaGetErrorString(e));           \
   exit(0); \
 }                                                                 \
}

inline __host__ __device__ float3 operator*(const float s, const float3 a)
{
	return make_float3(a.x * s, a.y * s, a.z * s);
}

inline __host__ __device__ float3 operator*(const float3 a, const float s)
{
	return make_float3(a.x * s, a.y * s, a.z * s);
}

inline __host__ __device__ float3 operator/(const float3 a, const float b)
{
	return make_float3(a.x / b, a.y / b, a.z / b);
}

inline __host__ __device__ float3 operator-(const float3 a, const float3 b)
{
	return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}


inline __host__ __device__ void operator*=(float3 &a, float s)
{
	a.x *= s; a.y *= s; a.z *= s;
}

inline __host__ __device__ float3 operator+(const float3 &a, const float3 &b)
{
	return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline __host__ __device__ float3 operator/(const float3 &a, const float3 &b)
{
	return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}



namespace CudaUtils
{
	const float3 CUDA_GRAVITY_ACCEL = make_float3(0, -9.81f * 1.7f, -10.1f);
	const int THREADS_PER_BLOCK = 32;



	template<typename T>
	struct CudaVec
	{
		T *m_data;
		size_t m_count;

		T& operator[] (const size_t x) const
		{
			return m_data[x];
		}
	};

	inline glm::vec3 MakeVec(const float3 a)
	{
		return glm::vec3(a.x, a.y, a.z);
	}

	inline __device__ size_t MyID()
	{
		return blockIdx.x * blockDim.x + threadIdx.x;
	}

	inline __device__ float3 normalize(const float3 a)
	{
		if (norm3df(a.x, a.y, a.z) == 0.f)
			printf("WAAAAAT (%f, %f, %f)\n", a.x, a.y, a.z);
		return a / norm3df(a.x, a.y, a.z);
	}

	inline __device__ float distance(const float3 a, const float3 b)
	{
		return norm3df(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	inline __device__ float min3(const float a, const float b, const float c)
	{
		return fminf(a, fminf(b, c));
	}

	inline __device__ float3 minf3(const float3 a, const float3 b)
	{
		return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
	}

	inline __device__ float3 min3(const float3 a, const float3 b, const float3 c)
	{
		return minf3(a, minf3(b, c));
	}

	inline __device__ float max3(const float a, const float b, const float c)
	{
		return fmaxf(a, fmaxf(b, c));
	}

	inline __device__ float3 maxf3(const float3 a, const float3 b)
	{
		return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
	}

	inline __device__ float3 max3(const float3 a, const float3 b, const float3 c)
	{
		return maxf3(a, maxf3(b, c));
	}

	inline __device__ float3 cross(const float3 a, const float3 b)
	{
		return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
	}

	inline __device__ float dot(const float3 a, const float3 b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	inline __device__ float3 FaceNormal(const float3 a, const float3 b, const float3 c)
	{
		return CudaUtils::cross(b - a, c - a);
	}

	inline __device__ uint32_t ExpandBy2(uint32_t v)
	{
		/*x &= 0x000003ff;
		x = (x ^ (x << 16)) & 0xff0000ff;
		x = (x ^ (x << 8)) & 0x0300f00f;
		x = (x ^ (x << 4)) & 0x030c30c3;
		x = (x ^ (x << 2)) & 0x09249249;
		return x;*/

		v = (v * 0x00010001u) & 0xFF0000FFu;
		v = (v * 0x00000101u) & 0x0F00F00Fu;
		v = (v * 0x00000011u) & 0xC30C30C3u;
		v = (v * 0x00000005u) & 0x49249249u;
		return v;
	}

	inline __device__ uint32_t Morton3D(const uint32_t x, const uint32_t y, const uint32_t z)
	{
		return (ExpandBy2(x) << 2) + (ExpandBy2(y) << 1) + ExpandBy2(z);
	}

	inline __device__ uint32_t Morton3D(const float3 position)
	{
		return Morton3D((uint32_t)position.x, (uint32_t)position.y, (uint32_t)position.z);
	}




	template<typename T>
	inline T *ToRaw(thrust::device_vector<T> &vec)
	{
		return thrust::raw_pointer_cast(&vec[0]);
	}
}