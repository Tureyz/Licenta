#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "CudaUtils.cuh"

#include "../Dependencies/cub/device/device_radix_sort.cuh"
//#include "../Dependencies/cub/device/device_reduce.cuh"
#include "../Dependencies/cub/device/device_select.cuh"


namespace CubWrap
{
	template <typename KeyT, typename ValueT>
	__forceinline__
		void SortByKey(thrust::device_vector<KeyT>& keys, thrust::device_vector<ValueT>& values, void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;

		KeyT *rawKey = thrust::raw_pointer_cast(&keys[0]);
		ValueT *rawValue = thrust::raw_pointer_cast(&values[0]);


		cub::DeviceRadixSort::SortPairs(NULL, requiredSize, rawKey, rawKey, rawValue, rawValue, keys.size());

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceRadixSort::SortPairs(tempStorage, requiredSize, rawKey, rawKey, rawValue, rawValue, keys.size());
	}

	template <typename T, typename SelectorT>
	__forceinline__
	void SelectInPlace(thrust::device_vector<T> &inVec, const SelectorT selector, int &selectedCount, void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;

		int *dSelectedCount;
		cudaMalloc(&dSelectedCount, sizeof(int));



		Physics::AABBCollision *rawVec = thrust::raw_pointer_cast(&inVec[0]);

		cub::DeviceSelect::If(NULL, requiredSize, rawVec, rawVec, dSelectedCount, inVec.size(), selector);

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceSelect::If(tempStorage, requiredSize, rawVec, rawVec, dSelectedCount, inVec.size(), selector);



		cudaMemcpy(&selectedCount, dSelectedCount, sizeof(int), cudaMemcpyDeviceToHost);
	}
}
