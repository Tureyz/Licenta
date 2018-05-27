#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "CudaUtils.cuh"


#ifndef __INTELLISENSE__

	#include "../Dependencies/cub/device/device_radix_sort.cuh"
	#include "../Dependencies/cub/device/device_reduce.cuh"
	#include "../Dependencies/cub/device/device_select.cuh"
	#include "../Dependencies/cub/device/device_run_length_encode.cuh"
	#include "../Dependencies/cub/device/device_scan.cuh"
#endif

namespace CubWrap
{
	template<typename T>
	__forceinline__
		T ReduceSum(const thrust::device_vector<T> &keys, void *& tempStorage, uint64_t &tempStorageSize)
	{
		T result;

		T *dSum;
		cudaMalloc(&dSum, sizeof(T));

		const T *rawKey = cu::raw(keys);

		uint64_t requiredSize = 0;

		cub::DeviceReduce::Sum(NULL, requiredSize, rawKey, dSum, (int)keys.size());
		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);
		cub::DeviceReduce::Sum(tempStorage, requiredSize, rawKey, dSum, (int)keys.size());

		cudaMemcpy(&result, dSum, sizeof(T), cudaMemcpyDeviceToHost);

		cudaFree(dSum);

		return result;
	}

	__forceinline__
		void ExclusiveSumInPlace(thrust::device_vector<int> &keys, const uint64_t actualKeysSize, void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;

		int *rawKey = thrust::raw_pointer_cast(&keys[0]);

		cub::DeviceScan::ExclusiveSum(NULL, requiredSize, rawKey, rawKey, (int) actualKeysSize);

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceScan::ExclusiveSum(tempStorage, requiredSize, rawKey, rawKey, (int) actualKeysSize);
	}


	__forceinline__
		void RLE(const thrust::device_vector<uint32_t> &keys, const int keysCount, thrust::device_vector<uint32_t> &uniques,
			thrust::device_vector<int> &counts, int &runCount,
			void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;

		const uint32_t *rawKey = thrust::raw_pointer_cast(&keys[0]);
		uint32_t *rawUnique = thrust::raw_pointer_cast(&uniques[0]);
		int *rawCount = thrust::raw_pointer_cast(&counts[0]);

		int *dRunCount;
		cudaMalloc(&dRunCount, sizeof(int));

		cub::DeviceRunLengthEncode::Encode(NULL, requiredSize, rawKey, rawUnique, rawCount, dRunCount, keysCount);

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceRunLengthEncode::Encode(tempStorage, requiredSize, rawKey, rawUnique, rawCount, dRunCount, keysCount);


		cudaMemcpy(&runCount, dRunCount, sizeof(int), cudaMemcpyDeviceToHost);

		cudaFree(dRunCount);
	}

	template <typename KeyT, typename ValueT>
	__forceinline__
		void SortByKey(Physics::DoubleBuffer<KeyT>& keys, Physics::DoubleBuffer<ValueT>& values, const int count, void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;


		KeyT *rawKeyCrt = thrust::raw_pointer_cast(&keys.buffers[keys.selector][0]);
		KeyT *rawKeyAlt = thrust::raw_pointer_cast(&keys.buffers[(keys.selector + 1) % 2][0]);

		ValueT *rawValueCrt = thrust::raw_pointer_cast(&values.buffers[values.selector][0]);
		ValueT *rawValueAlt = thrust::raw_pointer_cast(&values.buffers[(values.selector + 1) % 2][0]);

		cub::DeviceRadixSort::SortPairs(NULL, requiredSize, rawKeyCrt, rawKeyAlt, rawValueCrt, rawValueAlt, count);

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceRadixSort::SortPairs(tempStorage, requiredSize, rawKeyCrt, rawKeyAlt, rawValueCrt, rawValueAlt, count);

		keys.selector = (keys.selector + 1) % 2;
		values.selector = (values.selector + 1) % 2;
	}

	template <typename KeyT, typename ValueT>
	__forceinline__
		void SortByKey(thrust::device_vector<KeyT>& keys, thrust::device_vector<ValueT>& values, const int count, void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;

		KeyT *rawKey = thrust::raw_pointer_cast(&keys[0]);
		ValueT *rawValue = thrust::raw_pointer_cast(&values[0]);


		cub::DeviceRadixSort::SortPairs(NULL, requiredSize, rawKey, rawKey, rawValue, rawValue, count);

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceRadixSort::SortPairs(tempStorage, requiredSize, rawKey, rawKey, rawValue, rawValue, count);
	}

	template <typename T, typename SelectorT>
	__forceinline__
		void SelectInPlace(thrust::device_vector<T> &inVec, const SelectorT selector, int &selectedCount, void *& tempStorage, uint64_t &tempStorageSize)
	{
		uint64_t requiredSize = 0;

		int *dSelectedCount;
		cudaMalloc(&dSelectedCount, sizeof(int));



		T *rawVec = thrust::raw_pointer_cast(&inVec[0]);

		cub::DeviceSelect::If(NULL, requiredSize, rawVec, rawVec, dSelectedCount, inVec.size(), selector);

		CudaUtils::TempStorageGrow(tempStorage, tempStorageSize, requiredSize);

		cub::DeviceSelect::If(tempStorage, requiredSize, rawVec, rawVec, dSelectedCount, inVec.size(), selector);



		cudaMemcpy(&selectedCount, dSelectedCount, sizeof(int), cudaMemcpyDeviceToHost);

		cudaFree(dSelectedCount);
	}
}
