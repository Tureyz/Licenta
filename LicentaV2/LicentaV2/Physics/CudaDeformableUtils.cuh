#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "Structs.h"
#include "../Rendering/VertexFormat.h"



namespace DeformableUtils
{
	const float3 WORLD_MIN = make_float3(0, 0, 0);
	const float3 WORLD_MAX = make_float3(1, 1, 1);
	const float3 WIND_DIR = make_float3(0, 0, 0.2f);

	struct EqualToValue
	{
		uint64_t value;

		__host__ __device__ __forceinline__ EqualToValue(uint64_t val) : value(val) {}
		__host__ __device__ __forceinline__ bool operator()(const Physics::AABBCollision &a) const { return a.m_timestamp == value; }
	};

	struct PositiveCollisionTest
	{
		__host__ __device__ __forceinline__ bool operator()(const Physics::PrimitiveContact &a) const { return a.contact; }
	};



	bool CheckMortonDuplicates(const thrust::device_vector<unsigned int> mortonCodes);

	void CreateTriangles(const thrust::device_vector<float3> &positions, const thrust::device_vector<unsigned int> &indices, thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);

	__global__ void _CreateTriangles(const int triangleCount, const float3 * __restrict__ positions, const unsigned int * __restrict__ indices, Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);


	void UpdateTriangles(const thrust::device_vector<float3> &positions, thrust::device_vector<Physics::CudaTriangle> &triangles,
		thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<float3> &aabbMins, thrust::device_vector<float3> &aabbMaxs, const float thickness);//, thrust::device_vector<int> &colSizes);

	__global__ void _UpdateTriangles(const int triangleCount, const float3 * __restrict__ positions, Physics::CudaTriangle * __restrict__ triangles,
		uint64_t * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs, const float thickness);

	void UpdateVertexNormals(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &rawVNs, thrust::device_vector<uint32_t> &rawVNIDs,
		thrust::device_vector<float3> &accVNs);

	__global__ void CreateVNs(const int triangleCount, const Physics::CudaTriangle *__restrict__ triangles, float3 *__restrict__ rawVNs, uint32_t *__restrict__ rawVNIDs);

	__global__ void AccumulateVNs(const int rawVNCount, const float3 *__restrict__ rawVNs, const uint32_t *__restrict__ rawVNIDs, float3 *__restrict__ accumulatedVNs);

	__global__ void NormalizeVNs(const int particleCount, float3 * __restrict__ accVNs);



	void WorldConstraints(thrust::device_vector<float3> &positions);

	__global__ void _WorldConstraints(const int particleCount, float3 * __restrict__ positions, const float3 worldMin, const float3 worldMax);

	void AddWind(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<float3> &positions, const thrust::device_vector<float> &masses,
		const float timeStep);

	__global__ void _AddWind(const int triangleCount, const Physics::CudaTriangle * __restrict__ triangles, float3 * __restrict__ positions, const float * __restrict__ masses,
		const float3 windDirection, const float timeStep);


	void FinalVerticesUpdate(thrust::device_vector<Rendering::VertexFormat> &verts, const thrust::device_vector<Physics::CudaTriangle> &triangles,
		const thrust::device_vector<float3> &positions, const thrust::device_vector<float3>& vertexNormals);

	__global__ void _FinalVerticesUpdate(const int particleCount, Rendering::VertexFormat * __restrict__ verts, const Physics::CudaTriangle * __restrict__ triangles,
		const float3 * __restrict__ positions, const float3 * __restrict__ vertexNormals);


	void SortMortons(thrust::device_vector<uint64_t> &mortonCodes, thrust::device_vector<Physics::CudaTriangle> &triangles, void *& tempStorage, uint64_t &tempStorageSize);


	void ColorCollidingFeatures(thrust::device_vector<Rendering::VertexFormat> &verts,
		const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
		const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize);

	__global__ void ResetVertColors(const int vertsSize, Rendering::VertexFormat * __restrict__ verts);

	__global__ void ColorCollidingFeatures(Rendering::VertexFormat * __restrict__ verts,
		const Physics::PrimitiveContact * __restrict__ vfContacts, const uint64_t vfContactsSize,
		const Physics::PrimitiveContact * __restrict__ eeContacts, const uint64_t eeContactsSize);

	void CreateTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, thrust::device_vector<Physics::AABBCollision> &rawAABBCols,		
		const uint64_t timestamp, thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, void *&tempStorage, uint64_t &tempStorageSize);

	__global__ void CreateTriangleTests(const int filteredContactsSize, const Physics::CudaTriangle * __restrict__ triangles, const Physics::AABBCollision * __restrict__ filteredCols,
		Physics::PrimitiveContact * __restrict__ vfContacts, Physics::PrimitiveContact * __restrict__ eeContacts);

	void DCDTriangleTests(const thrust::device_vector<Physics::CudaTriangle> &triangles, const thrust::device_vector<float3> &positions,
		thrust::device_vector<Physics::PrimitiveContact> &vfContacts, uint64_t &vfContactsSize,
		thrust::device_vector<Physics::PrimitiveContact> &eeContacts, uint64_t &eeContactsSize, const float thickness, void *&tempStorage, uint64_t &tempStorageSize);


	void CreateImpulses(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &velocities,
		const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
		const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize,
		thrust::device_vector<uint32_t> &impulseIDs, thrust::device_vector<float3> &impulseValues, uint64_t &impulsesSize,
		thrust::device_vector<uint32_t> &impulseRLEUniques, thrust::device_vector<int> &impulseRLECounts, int &impulseRunCount,
		thrust::device_vector<float3> &accumulatedImpulses, const float stiffness, const float vertexMass, const float timeStep, const float thickness,
		void *&tempStorage, uint64_t &tempStorageSize);

	void CreateImpulses(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &velocities,
		const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
		const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize,
		Physics::DoubleBuffer<uint32_t> &impulseIDs, Physics::DoubleBuffer<float3> &impulseValues, uint64_t &impulsesSize,
		thrust::device_vector<uint32_t> &impulseRLEUniques, thrust::device_vector<int> &impulseRLECounts, int &impulseRunCount,
		thrust::device_vector<float3> &accumulatedImpulses, const float stiffness, const float vertexMass, const float timeStep, const float thickness,
		void *&tempStorage, uint64_t &tempStorageSize);


	__global__ void _CreateImpulses(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
		const Physics::PrimitiveContact * __restrict__ vfs, const uint64_t vfSize,
		const Physics::PrimitiveContact * __restrict__ ees, const uint64_t eeSize,
		uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, const float stiffness, const float vertexMass, const float timeStep, const float thickness);

	__device__ void _CreateImpulseVF(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
		const Physics::PrimitiveContact & contact, uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, const int myImpulseStart, const float stiffness,
		const float vertexMass, const float timeStep, const float thickness);

	__device__ void _CreateImpulseEE(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
		const Physics::PrimitiveContact & contact, uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, const int myImpulseStart, const float stiffness,
		const float vertexMass, const float timeStep, const float thickness);

	__device__ void _CreateImpulse(const float3 * __restrict__ positions, const Physics::PrimitiveContact & contact, const float3 &vr,
		uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, const int myImpulseStart,
		const float stiffness, const float vertexMass, const float timeStep, const float thickness);

	__global__ void AccumulateImpulses(const int impulseRunCount, 
		const uint32_t * __restrict__ impulseRLEUniques, const int * __restrict__ impulseRLEPrefixSums,
		const float3 * __restrict__ impulseValues, const uint64_t impulseValuesSize, float3 * __restrict__ accumulatedImpulses);


	void ApplyImpulses(thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &prevPositions,
		thrust::device_vector<float3> &velocities,
		const thrust::device_vector<float3> &impulses, const thrust::device_vector<bool> &fixedVerts, const float vertexMass, const float timeStep);

	__global__ void ApplyImpulses(const int particleCount, float3 * __restrict__ positions, const float3 * __restrict__ prevPositions, float3 * __restrict__ velocities,
		const float3 * __restrict__ impulses, const bool * __restrict__ fixedVerts, const float vertexMass, const float timeStep);

	//void FlattenAABBCollisionList(thrust::device_vector<Physics::AABBCollision*> &collisions, thrust::device_vector<int> &collisionSizes,
	//	thrust::device_vector<Physics::AABBCollision> &output, void *&tempStorage, uint64_t &tempStorageSize);

	//__global__ void _FlattenAABBCollisionList(const int threadCount, Physics::AABBCollision ** __restrict__ collisions, int * __restrict__ collisionSizes,
	//	Physics::AABBCollision * __restrict__ output);
}