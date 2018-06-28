#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
//#include <math_functions.h>
#include "../Dependencies/glm/glm.hpp"

#include "../Physics/Structs.h"



//
//typedef float * __restrict__ fp;
//typedef float3 * __restrict__ f3p;
//typedef int * __restrict__ ip;
//
//typedef const float * __restrict__ cfp;
//typedef const float3 * __restrict__ cf3p;
//typedef const int * __restrict__ cip;


//#define CUDA_ERROR_CHECK
#define cudaCheckError()    __cudaCheckError( __FILE__, __LINE__ )

inline void __cudaCheckError(const char *file, const int line)
{
#ifdef CUDA_ERROR_CHECK
	cudaError err = cudaGetLastError();
	if (cudaSuccess != err)
	{
		fprintf(stderr, "cudaCheckError() failed at %s:%i : %s\n",
			file, line, cudaGetErrorString(err));
		exit(-1);
	}

	// More careful checking. However, this will affect performance.
	// Comment away if needed.
	//err = cudaDeviceSynchronize();
	//if (cudaSuccess != err)
	//{
	//	fprintf(stderr, "cudaCheckError() with sync failed at %s:%i : %s\n",
	//		file, line, cudaGetErrorString(err));
	//	exit(-1);
	//}
#endif

	return;
}


__host__ __device__ float3 operator*(const float3 &a, const float3 &b);
__host__ __device__ float3 operator*(const float s, const float3 &a);
__host__ __device__ float3 operator*(const float3 &a, const float s);
__host__ __device__ float3 operator/(const float3 &a, const float b);
__host__ __device__ float3 operator-(const float3 &a, const float3 &b);
__host__ __device__ float3 operator-(const float3 &a, const float b);
__host__ __device__ float3 operator+(const float3 &a, const float b);
__host__ __device__ void operator*=(float3 &a, const float s);
__host__ __device__ void operator/=(float3 &a, const float s);
__host__ __device__ float3 operator+(const float3 &a, const float3 &b);
__host__ __device__ float3 operator/(const float3 &a, const float3 &b);

__host__ __device__ void operator+=(float3 &a, const float3 &b);

__host__ __device__ bool operator==(const float3 &a, const float3 &b);
__host__ __device__ bool operator!=(const float3 &a, const float3 &b);

__host__ __device__ float3 operator-(const float3 &a);


namespace CudaUtils
{

	struct CudaTimer
	{
		cudaEvent_t m_start;
		cudaEvent_t m_end;

		~CudaTimer();
		CudaTimer();
		void Start();
		float End();
	};

	//const float3 CUDA_GRAVITY_ACCEL = make_float3(0, -0.981f, -0.0f);
	const int THREADS_PER_BLOCK = 256;


	std::string MemUsage(const double freeInit = -1.0);

	double VRAMUsage();

	glm::vec3 MakeVec(const float3 &a);


	__host__ __device__ bool isZero(const float3 &vec);

	__device__ float3 ProjectOnPlane(const float3 &point, const float3 &planePoint, const float3 &planeNormal);

	__device__ float3 ProjectOnLine(const float3 &point, const float3 &edgeP1, const float3 &edgeP2);

	// Assumes point is on P1, P2 already
	__device__ float3 ClampOnLine(const float3 &point, const float3 &edgeP1, const float3 &edgeP2);

	__device__ bool isBetween(const float value, const float min, const float max);

	__device__ bool isBetweenExclusive(const float value, const float min, const float max);

	__host__ __device__ float clamp(const float value, const float min, const float max);

	__host__ __device__ float3 clamp(const float3 &value, const float3 &min, const float3 &max);

	__device__ bool isNan(const float3 &a);

	__device__ size_t MyID();

	__device__ float3 normalize(const float3 &a);

	__device__ float len(const float3 &a);

	__device__ float distance(const float3 &a, const float3 &b);

	__device__ float min3(const float a, const float b, const float c);

	__device__ float3 minf3(const float3 &a, const float3 &b);

	__device__ float3 min3(const float3 &a, const float3 &b, const float3 &c);

	__device__ float max3(const float a, const float b, const float c);

	__device__ float3 maxf3(const float3 &a, const float3 &b);

	__device__ float3 max3(const float3 &a, const float3 &b, const float3 &c);

	__device__ float3 cross(const float3 &a, const float3 &b);

	__device__ float dot(const float3 &a, const float3 &b);

	__device__ float3 reflect(const float3 &vec, const float3 &normal);

	__device__ void tangentNormal(const float3 &vec, const float3 &normal, float3 &outTangent, float3 & outNormal);


	__device__ float3 FaceNormal(const float3 &a, const float3 &b, const float3 &c);

	__device__ uint64_t ExpandBy2(uint64_t v);

	//__device__ uint32_t ExpandBy2(uint32_t v);

	//__device__ uint32_t Morton3D(const uint32_t x, const uint32_t y, const uint32_t z);

	__device__ uint64_t Morton3D(const float x, const float y, const float z);

	//__device__ uint32_t Morton3D(const float3 position);

	__device__ uint64_t Morton3D64(const float3 position);

	__device__ int ComPref(const unsigned int mortonA, const unsigned int mortonB, const int indexA, const int indexB, const int max);

	__device__ int Delta(const uint64_t *mortonCodes, const int i, const int j, const int max);

	__host__ __device__ int sgn(int a);

	__device__ bool AABBOverlap(const float3 aMin, const float3 aMax, const float3 bMin, const float3 bMax);


	__device__ void PushBack(const Physics::AABBCollision &element, Physics::AABBCollision *&array, const int crtId, int &size);

	__device__ void CheckResize(Physics::AABBCollision *&array, const int crtId, int &size);


	__device__ const float3 AdvancePositionInTime(const float3 &position, const float3 &velocity, const float time);

	__device__ const float3 AdvancePositionInTimePos(const float3 &prevPosition, const float3 &position, const float time);

	__device__ void SetBit(char &byte, const int bitIndex);

	__device__ void ClearBit(char &byte, const int bitIndex);

	__device__ bool GetBit(const char byte, const int bitIndex);

	__device__ void SetLength(float3 &v, const float length);

	__device__ void ClampLength(float3 &v, const float min, const float max);

	void TempStorageGrow(void *&storage, uint64_t &size, const uint64_t requiredSize);
	//template<typename T>
	//T *ToRaw(thrust::device_vector<T> &vec);
}

namespace cu
{
	template <typename T>
	__forceinline__
		T *raw(thrust::device_vector<T> &vec) { return thrust::raw_pointer_cast(&vec[0]); }

	template <typename T>
	__forceinline__
		const T *raw(const thrust::device_vector<T> &vec) { return thrust::raw_pointer_cast(&vec[0]); }

	__forceinline__ int nb(const int threadCount) { return (threadCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;	}
}
