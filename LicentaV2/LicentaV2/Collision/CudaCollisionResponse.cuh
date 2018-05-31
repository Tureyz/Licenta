#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"

namespace DeformableUtils
{

	void CreateImpulses(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &velocities,
		const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
		const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize,
		Physics::DoubleBuffer<uint32_t> &impulseIDs, Physics::DoubleBuffer<float3> &impulseValues, thrust::device_vector<bool> &impulseFlags, uint64_t &impulsesSize,
		thrust::device_vector<uint32_t> &impulseRLEUniques, thrust::device_vector<int> &impulseRLECounts, int &impulseRunCount,
		thrust::device_vector<float3> &accumulatedImpulses, const float stiffness, const float vertexMass, const float timeStep, const float thickness,
		void *&tempStorage, uint64_t &tempStorageSize);


	__global__ void _CreateImpulses(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
		const Physics::PrimitiveContact * __restrict__ vfs, const uint64_t vfSize,
		const Physics::PrimitiveContact * __restrict__ ees, const uint64_t eeSize,
		uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, bool * __restrict__ impulseFlags, 
		const float stiffness, const float vertexMass, const float timeStep, const float thickness);

	__device__ void _CreateImpulseVF(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
		const Physics::PrimitiveContact & contact, uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, bool * __restrict__ impulseFlags,
		const int myImpulseStart, const float stiffness,
		const float vertexMass, const float timeStep, const float thickness);

	__device__ void _CreateImpulseEE(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
		const Physics::PrimitiveContact & contact, uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, bool * __restrict__ impulseFlags,
		const int myImpulseStart, const float stiffness,
		const float vertexMass, const float timeStep, const float thickness);

	__device__ void _CreateImpulse(const float3 * __restrict__ positions, const Physics::PrimitiveContact & contact, const float3 &vr,
		uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, bool * __restrict__ impulseFlags, const int myImpulseStart,
		const float stiffness, const float vertexMass, const float timeStep, const float thickness);

	__global__ void AccumulateImpulses(const int impulseRunCount,
		const uint32_t * __restrict__ impulseRLEUniques, const int * __restrict__ impulseRLEPrefixSums,
		const float3 * __restrict__ impulseValues, const uint64_t impulseValuesSize, float3 * __restrict__ accumulatedImpulses);


	void ApplyImpulses(thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &prevPositions,
		thrust::device_vector<float3> &velocities,
		const thrust::device_vector<float3> &impulses, const thrust::device_vector<bool> &fixedVerts, const float vertexMass, const float timeStep);

	__global__ void ApplyImpulses(const int particleCount, float3 * __restrict__ positions, const float3 * __restrict__ prevPositions, float3 * __restrict__ velocities,
		const float3 * __restrict__ impulses, const bool * __restrict__ fixedVerts, const float vertexMass, const float timeStep);


	//void CreateImpulses(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &velocities,
	//	const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
	//	const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize,
	//	thrust::device_vector<uint32_t> &impulseIDs, thrust::device_vector<float3> &impulseValues, thrust::device_vector<bool> &impulseFlags, uint64_t &impulsesSize,
	//	thrust::device_vector<uint32_t> &impulseRLEUniques, thrust::device_vector<int> &impulseRLECounts, int &impulseRunCount,
	//	thrust::device_vector<float3> &accumulatedImpulses, const float stiffness, const float vertexMass, const float timeStep, const float thickness,
	//	void *&tempStorage, uint64_t &tempStorageSize);
}