#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"


namespace CudaBVH
{

	void GetAABBCollisions(const thrust::device_vector<int> &lefts, const thrust::device_vector<int> &rights,
		const thrust::device_vector<float3> &AABBMins, const thrust::device_vector<float3> &AABBMaxs,
		const thrust::device_vector<int> &rmlls, const thrust::device_vector<int> &rmlrs,
		thrust::device_vector<Physics::AABBCollision> &collisions, thrust::device_vector<bool> &flags, const int chunkSize, const uint64_t timestamp);

	__global__ void _GetAABBCollisions(const int leafCount, const int * __restrict__ lefts, const int * __restrict__ rights,
		const float3 * __restrict__ aabbMins, const float3 * __restrict__ aabbMaxs, const int * __restrict__ rmlls, const int * __restrict__ rmlrs,
		Physics::AABBCollision * __restrict__ collisions, bool * __restrict__ flags, const int chunkSize, const uint64_t timestamp);

	void ComputeTreeAABBs(const thrust::device_vector<int> &lefts, const thrust::device_vector<int> &rights,
		const thrust::device_vector<int> &parents, thrust::device_vector<int> &nodesVisited,
		thrust::device_vector<float3> &AABBMins, thrust::device_vector<float3> &AABBMaxs, thrust::device_vector<int> &rmlls, thrust::device_vector<int> &rmlrs);

	static __global__ void ComputeTreeAABBs(const int leafNodeCount, const int * __restrict__ lefts, const int * __restrict__ rights,
		const int * __restrict__ parents, int * __restrict__ nodesVisited,
		float3 * __restrict__ AABBMins, float3 * __restrict__ AABBMaxs, int * __restrict__ rmlls, int * __restrict__ rmlrs);

	void GenerateBVH2(const thrust::device_vector<uint64_t> &sortedMortons, thrust::device_vector<int> &lefts,
		thrust::device_vector<int> &rights, thrust::device_vector<int> &parents, thrust::device_vector<int> &nodesVisited);

	static __global__ void _GenerateBVH2(const int internalNodeCount, const uint64_t * __restrict__ sortedMortons,
		int * __restrict__ lefts, int * __restrict__ rights, int * __restrict__ parents,
		int * __restrict__ nodesVisited);

	static __device__ int2 FindRange(const uint64_t * __restrict__ sortedMortons, const int count, const int id);
	static __device__ int FindSplit(const uint64_t * __restrict__ sortedMortons, const int first, const int last);

	void BVHTestUnit(thrust::device_vector<uint64_t> &sortedMortons, const thrust::device_vector<int> &lefts,
		const thrust::device_vector<int> &rights, const thrust::device_vector<int> &parents,
		const thrust::device_vector<int> &nodesVisited, const thrust::device_vector<float3> &AABBMins,
		const thrust::device_vector<float3> &AABBMaxs);	

	void PrintTree(const thrust::device_vector<int> &lefts, const thrust::device_vector<int> &rights,
		const thrust::device_vector<int> &parents, const thrust::device_vector<int> &rmlls, const thrust::device_vector<int> &rmlrs);

	static __global__ void PrintTree(const int leafNodeCount, const int * __restrict__ lefts, const int * __restrict__ rights,
		const int * __restrict__ parents, const int * __restrict__ rmlls, const int * __restrict__ rmlrs);


	//void GenerateBVH(const thrust::device_vector<unsigned int> &sortedMortons, thrust::device_vector<int> &lefts,
	//	thrust::device_vector<int> &rights, thrust::device_vector<int> &parents, thrust::device_vector<int> &nodesVisited);

	//static __global__ void _GenerateBVH(const int internalNodeCount, const unsigned int * __restrict__ sortedMortons,
	//	 int * __restrict__ lefts, int * __restrict__ rights, int * __restrict__ parents,
	//	int * __restrict__ nodesVisited);

	//static __device__ int BinarySearch(const int start, const int del, const unsigned int * __restrict__ sortedMortons,
	//	const int id, const int d, const int internalNodeCount);

	//static __device__ int2 FindRange(const unsigned int * __restrict__ sortedMortons, const int id, const int d,
	//	const int internalNodeCount);
	//static __device__ int FindSplit(const unsigned int * __restrict__ sortedMortons, const int id, const int d,
	//	const int j, const int l, const int internalNodeCount);



}