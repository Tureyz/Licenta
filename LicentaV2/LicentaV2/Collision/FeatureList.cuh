#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "../Physics/Structs.h"
#include "../Rendering/VertexFormat.h"

namespace FeatureList
{
	struct FeatureList
	{

		void Init(const std::vector<Rendering::VertexFormat> &verts,
			const std::vector<unsigned int>  &indices, const Physics::ClothParams params);


		void UpdateFeatureAABBs(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &prevPositions);

		void AssignFeatures(thrust::device_vector<Physics::CudaTriangle> &triangles);

		Physics::ClothParams m_parentParams;

		int m_particleCount;
		thrust::device_vector<float3> m_particleAABBMins;
		thrust::device_vector<float3> m_particleAABBMaxs;


		int m_edgeCount;
		thrust::device_vector<int> m_edgeMap;
		thrust::device_vector<int> m_edgev1s;
		thrust::device_vector<int> m_edgev2s;
		thrust::device_vector<float3> m_edgeAABBMins;
		thrust::device_vector<float3> m_edgeAABBMaxs;


		//thrust::device_vector<Physics::CudaTriangle> m_dTriangles;
		//thrust::device_vector<uint64_t> m_dMortonCodes;
		//thrust::device_vector<float3> m_dAABBMins;
		//thrust::device_vector<float3> m_dAABBMaxs;

	private:
		int lin(const int a, const int b);

	};
}

__global__ void _AssignFeatures(const int particleCount, const int triangleCount, Physics::CudaTriangle * __restrict__ triangles,
	int * __restrict__ assignedParticles, int * __restrict__ assignedEdges);


__global__ void UpdateParticleAABBs(const int particleCount, const float3 * __restrict__ positions,
	const float3 * __restrict__ prevPositions, const float thickness,
	float3 * __restrict__ particleMins, float3 * __restrict__ particleMaxs);

__global__ void UpdateEdgeAABBs(const int edgeCount, const float3 * __restrict__ particleMins,
	const float3 * __restrict__ particleMaxs,
	const int * __restrict__ edgev1s, const int * __restrict__ edgev2s, const float thickness, 
	float3 * __restrict__ edgeMins, float3 * __restrict__ edgeMaxs);


