#include "FeatureList.cuh"

#include <thrust/host_vector.h>

#include "../Core/CudaUtils.cuh"

int FeatureList::FeatureList::lin(const int a, const int b)
{
	return a * m_parentParams.dims.x + b;
}

void FeatureList::FeatureList::Init(const std::vector<Rendering::VertexFormat> &verts,
	const std::vector<unsigned int>  &indices, const Physics::ClothParams params)
{
	m_parentParams = params;
	m_particleCount = (int) verts.size();

	m_particleAABBMins.resize(m_particleCount);
	m_particleAABBMaxs.resize(m_particleCount);


	thrust::host_vector<int> hEv1s;
	thrust::host_vector<int> hEv2s;
	thrust::host_vector<int> edgeMap(m_particleCount * m_particleCount, -1);

	for (int i = 0; i < indices.size(); i += 3)
	{
		for (int j = 0; j < 3; ++j)
		{
			int id1 = indices[i + (j % 3)];
			int id2 = indices[i + ((j + 1) % 3)];

			int min = id1 < id2 ? id1 : id2;
			int max = id1 + id2 - min;

			int eid = min * m_particleCount + max;

			if (edgeMap[eid] == -1)
			{
				hEv1s.push_back(min);
				hEv2s.push_back(max);
				edgeMap[eid] = (int) hEv1s.size() - 1;
			}

		}
	}

	m_edgeCount = (int) hEv1s.size();
	m_edgeMap = edgeMap;
	m_edgev1s = hEv1s;
	m_edgev2s = hEv2s;
	m_edgeAABBMins.resize(m_edgeCount);
	m_edgeAABBMaxs.resize(m_edgeCount);

}

void FeatureList::FeatureList::UpdateFeatureAABBs(const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& prevPositions)
{
	int numBlocks = cu::nb(m_particleCount);

	UpdateParticleAABBs << < numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_particleCount, cu::raw(positions), cu::raw(prevPositions),
		m_parentParams.thickness, cu::raw(m_particleAABBMins), cu::raw(m_particleAABBMaxs));

	numBlocks = cu::nb(m_edgeCount);

	UpdateEdgeAABBs << < numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_edgeCount, cu::raw(m_particleAABBMins), cu::raw(m_particleAABBMaxs),
		cu::raw(m_edgev1s), cu::raw(m_edgev2s), m_parentParams.thickness,
		cu::raw(m_edgeAABBMins), cu::raw(m_edgeAABBMaxs));

}

void FeatureList::FeatureList::AssignFeatures(thrust::device_vector<Physics::CudaTriangle>& triangles)
{
	thrust::device_vector<int> assignedParticles(m_particleCount, 0);
	thrust::device_vector<int> assignedEdges(m_particleCount * m_particleCount, 0);


	int numBlocks = cu::nb((int) triangles.size());

	_AssignFeatures << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_particleCount, (int) triangles.size(), cu::raw(triangles), cu::raw(assignedParticles), cu::raw(assignedEdges));

}

__global__ void _AssignFeatures(const int particleCount, const int triangleCount, 
	Physics::CudaTriangle *__restrict__ triangles, int *__restrict__ assignedParticles, int *__restrict__ assignedEdges)
{
	const int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	const Physics::CudaTriangle myTriangle = triangles[id];
	const int v1 = myTriangle.m_v1;
	const int v2 = myTriangle.m_v2;
	const int v3 = myTriangle.m_v3;

	char myAssignments = 0;

	if (atomicExch(&assignedParticles[v1], 1) == 0)
		CudaUtils::SetBit(myAssignments, 5);

	if (atomicExch(&assignedParticles[v2], 1) == 0)
		CudaUtils::SetBit(myAssignments, 4);

	if (atomicExch(&assignedParticles[v3], 1) == 0)
		CudaUtils::SetBit(myAssignments, 3);


	int min = v1 < v2 ? v1 : v2;
	int max = v1 + v2 - min;

	int eid = min * particleCount + max;

	if (atomicExch(&assignedEdges[eid], 1) == 0)
		CudaUtils::SetBit(myAssignments, 2);

	min = v2 < v3 ? v2 : v3;
	max = v2 + v3 - min;
	eid = min * particleCount + max;

	if (atomicExch(&assignedEdges[eid], 1) == 0)
		CudaUtils::SetBit(myAssignments, 1);

	min = v3 < v1 ? v3 : v1;
	max = v3 + v1 - min;
	eid = min * particleCount + max;

	if (atomicExch(&assignedEdges[eid], 1) == 0)
		CudaUtils::SetBit(myAssignments, 0);


	triangles[id].m_assignedFeatures = myAssignments;
}

__global__ void UpdateParticleAABBs(const int particleCount, const float3 * __restrict__ positions,
	const float3 * __restrict__ prevPositions, const float thickness,
	float3 * __restrict__ particleMins, float3 * __restrict__ particleMaxs)
{
	const int id = CudaUtils::MyID();

	if (id >= particleCount)
		return;

	float3 pos = positions[id];
	float3 prevPos = prevPositions[id];

	particleMins[id] = CudaUtils::minf3(pos, prevPos) - thickness;
	particleMaxs[id] = CudaUtils::maxf3(pos, prevPos) + thickness;
}

__global__ void UpdateEdgeAABBs(const int edgeCount, const float3 * __restrict__ particleMins,
	const float3 * __restrict__ particleMaxs,
	const int * __restrict__ edgev1s, const int * __restrict__ edgev2s, const float thickness,
	float3 * __restrict__ edgeMins, float3 * __restrict__ edgeMaxs)
{
	const int id = CudaUtils::MyID();

	if (id >= edgeCount)
		return;

	edgeMins[id] = CudaUtils::minf3(particleMins[edgev1s[id]], particleMins[edgev2s[id]]);
	edgeMaxs[id] = CudaUtils::maxf3(particleMaxs[edgev1s[id]], particleMaxs[edgev2s[id]]);
}



