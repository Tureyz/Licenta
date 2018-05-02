#include "CudaDeformableBody.cuh"

#include <iostream>

#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>


#include "../Core/Utils.hpp"
#include "../Core/CudaUtils.cuh"
#include "../Core/TimeUtils.h"




int Physics::CudaDeformableBody::lin(const int i, const int j)
{
	return i * m_dims.first + j;
}

Physics::CudaDeformableBody::~CudaDeformableBody()
{

}


__global__ void CreateTriangles(const int triangleCount, const float3 * __restrict__ positions, const unsigned int * __restrict__ indices, int * __restrict__ v1s, int * __restrict__ v2s, int * __restrict__ v3s,
	float3 * __restrict__ faceNormals, unsigned int * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	int startID = id * 3;

	v1s[id] = indices[startID];
	v2s[id] = indices[startID + 1];
	v3s[id] = indices[startID + 2];

	float3 posV1 = positions[v1s[id]];
	float3 posV2 = positions[v2s[id]];
	float3 posV3 = positions[v3s[id]];

	faceNormals[id] = CudaUtils::FaceNormal(posV1, posV2, posV3);


	float3 min = CudaUtils::min3(posV1, posV2, posV3);
	float3 max = CudaUtils::max3(posV1, posV2, posV3);

	aabbMins[id] = min;
	aabbMaxs[id] = max;

	mortonCodes[id] = CudaUtils::Morton3D(min + (max - min) / 2.f);	
}

Physics::CudaDeformableBody::CudaDeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices, std::pair<int, int> dims)
{

	m_structuralStiffness = 0.9f;
	m_bendStiffness = 0.2f;
	m_shearStiffness = 0.6f;
	m_dampingCoefficient = 0.3f;

	m_objectMass = 10.5f;

	m_verts = verts;
	m_indices = indices;
	m_dims = dims;

	float vertexMass = m_objectMass / (m_dims.first * m_dims.second);

	thrust::host_vector<float3> initPos, initVel;
	thrust::host_vector<unsigned int> cudaIndices(indices->size());
	thrust::host_vector<float> masses;


	for (int i = 0; i < cudaIndices.size(); ++i)
	{
		cudaIndices[i] = (*indices)[i];
	}

	for (auto vert : *m_verts)
	{
		initPos.push_back(make_float3(vert.m_position.x, vert.m_position.y, vert.m_position.z));
		initVel.push_back(make_float3(Core::Utils::RandomAround(0.f, 0.02f), Core::Utils::RandomAround(0.f, 0.02f), Core::Utils::RandomAround(0.f, 0.02f)));
		
		//initVel.push_back(make_float3(0.f, 0.f, 0.f));
		//masses.push_back(1.0f);
		masses.push_back(vertexMass);
	}

	m_dVertexNormals.resize(initPos.size());

	//std::vector<std::vector<int>> auxSpringInfo(m_vertexPositions.size(), std::vector<int>());

	thrust::host_vector<thrust::host_vector<int>> springInfo(initPos.size());


	

	thrust::host_vector<int> aIDs;
	thrust::host_vector<int> bIDs;
	thrust::host_vector<float> ks;
	thrust::host_vector<float> l0s;

	for (int i = 0; i < m_dims.first; ++i)
	{
		for (int j = 0; j < m_dims.second; ++j)
		{			

			if (i + 1 < m_dims.first)
				AddSpring(i, j, i + 1, j, m_structuralStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);

			if (j + 1 < m_dims.first)
				AddSpring(i, j, i, j + 1, m_structuralStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);

			if (i + 1 < m_dims.first && j + 1 < m_dims.first)
			{
				AddSpring(i, j, i + 1, j + 1, m_shearStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);
				AddSpring(i + 1, j, i, j + 1, m_shearStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);
			}

			if (i + 2 < m_dims.first)			
				AddSpring(i, j, i + 2, j, m_bendStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);

			if (j + 2 < m_dims.first)
				AddSpring(i, j, i, j + 2, m_bendStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);

			if (i + 2 < m_dims.first && j + 2 < m_dims.first)
				AddSpring(i, j, i + 2, j + 2, m_bendStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);
		}
	}

	thrust::host_vector<int> springIDs(springInfo.size() + 1);
	springIDs[0] = 0;

	for (int i = 1; i < springInfo.size() + 1; ++i)
	{
		springIDs[i] = springIDs[i - 1] + springInfo[i - 1].size();
	}

	thrust::host_vector<float3> forces(initPos.size(), make_float3(0, 0, 0));
	thrust::host_vector<int> linSpringInfo;

	for (auto vec : springInfo)
	{
		linSpringInfo.insert(linSpringInfo.end(), vec.begin(), vec.end());
	}


	m_dPrevPositions = initPos;
	m_dPositions = initPos;
	m_dVelocities = initVel;
	m_dForces = forces;
	m_dMasses = masses;
	m_dSpringIDs = springIDs;
	m_dLinSpringInfo = linSpringInfo;

	m_daIDs = aIDs;
	m_dbIDs = bIDs;
	m_dks = ks;
	m_dl0s = l0s;

	m_particleCount = initPos.size();
	m_springCount = aIDs.size();

	m_fixedVerts.resize(m_particleCount);

	for (int i = 0; i < dims.first; ++i)
	{
		m_fixedVerts[i * dims.first] = true;
	}


	m_triangleCount = cudaIndices.size() / 3;

	m_dTriangleIDs.resize(m_triangleCount);
	thrust::sequence(m_dTriangleIDs.begin(), m_dTriangleIDs.end());
	m_dTriV1s.resize(m_triangleCount);
	m_dTriV2s.resize(m_triangleCount);
	m_dTriV3s.resize(m_triangleCount);
	m_dFaceNormals.resize(m_triangleCount);
	m_dMortonCodes.resize(m_triangleCount);
	m_dAABBMins.resize(m_triangleCount);
	m_dAABBMaxs.resize(m_triangleCount);

	

	thrust::device_vector<unsigned int> dCudaIndices = cudaIndices;


	int numBlocks = (m_triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;
	
	CreateTriangles<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(m_triangleCount, CudaUtils::ToRaw(m_dPositions), CudaUtils::ToRaw(dCudaIndices), CudaUtils::ToRaw(m_dTriV1s),
		CudaUtils::ToRaw(m_dTriV2s), CudaUtils::ToRaw(m_dTriV3s), CudaUtils::ToRaw(m_dFaceNormals), CudaUtils::ToRaw(m_dMortonCodes), CudaUtils::ToRaw(m_dAABBMins),
		CudaUtils::ToRaw(m_dAABBMaxs));


}

void Physics::CudaDeformableBody::AddSpring(const int i1, const int j1, const int i2, const int j2, const float stiffness,
	thrust::host_vector<int> &aIDs, thrust::host_vector<int> &bIDs, thrust::host_vector<float> &ks, thrust::host_vector<float> &l0s,
	thrust::host_vector<thrust::host_vector<int>> &springInfo, const thrust::host_vector<float3> &initPos)
{

	int id1 = lin(i1, j1);
	int id2 = lin(i2, j2);


	float l0 = glm::distance(CudaUtils::MakeVec(initPos[id1]), CudaUtils::MakeVec(initPos[id2]));
	
	aIDs.push_back(id1);
	bIDs.push_back(id2);
	ks.push_back(stiffness);
	l0s.push_back(l0);

	springInfo[id1].push_back(aIDs.size() - 1);

	aIDs.push_back(id2);
	bIDs.push_back(id1);
	ks.push_back(stiffness);
	l0s.push_back(l0);

	springInfo[id2].push_back(aIDs.size() - 1);
}

__device__ float ComputeSpringDeformation(const float3 a, const float3 b, const float l0)
{
	return (CudaUtils::distance(a, b) - l0) / l0;
}

__global__ void ClothEngineStep(const int particleCount, float3 * __restrict__ positions, float3 *__restrict__ prevPositions, float3 *__restrict__ velocities,
	const float *__restrict__ masses, const int springCount, const int *__restrict__ aIDs, const int *__restrict__ bIDs, const float *__restrict__ ks, const float *__restrict__ l0s, 
	const int *__restrict__ springInfo, const int *__restrict__ springIDs, const float3 gravity, const float dampingCoef, const float timeStep, const float bendCoef,
	const bool *__restrict__ fixedVerts)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount || fixedVerts[id])
		return;

	
	//float3 force = masses[id] * gravity - dampingCoef * velocities[id];
	float3 force = masses[id] * gravity - dampingCoef * (positions[id] - prevPositions[id]);

	for (int i = springIDs[id]; i < springIDs[id + 1]; ++i)
	{

		int sprID = springInfo[i];
		float3 dir = positions[aIDs[sprID]] - positions[bIDs[sprID]];

		//float len = norm3df(dir.x, dir.y, dir.z);

		//float k = ComputeSpringDeformation(positions[aIDs[sprID]], positions[bIDs[sprID]], l0s[sprID]) > 0.1f && ks[sprID] != bendCoef ? 0.99f : ks[sprID];

		force = force - ks[sprID] * (dir - l0s[sprID] * CudaUtils::normalize(dir));
		//printf("Force: (%f, %f, %f)\n", force.x, force.y, force.z);
		//force = force - k * (len - l0s[sprID]) * (dir / len);
	}

	float3 accel = force / masses[id];

	//Explicit Euler
	//velocities[id] = velocities[id] + timeStep * accel;
	//positions[id] = positions[id] + timeStep * velocities[id];


	// Verlet
	
	float3 newPos = 2.f * positions[id] - prevPositions[id] + accel * timeStep * timeStep;
	prevPositions[id] = positions[id];
	positions[id] = newPos;

}


__global__ void UpdateTriangles(const int triangleCount, const float3 * __restrict__ positions, const int * __restrict__ v1s, const int * __restrict__ v2s, const int * __restrict__ v3s,
	float3 * __restrict__ faceNormals, unsigned int * __restrict__ mortonCodes, float3 * __restrict__ aabbMins, float3 * __restrict__ aabbMaxs)
{
	int id = CudaUtils::MyID();
	if (id >= triangleCount)
		return;

	float3 posV1 = positions[v1s[id]];
	float3 posV2 = positions[v2s[id]];
	float3 posV3 = positions[v3s[id]];

	faceNormals[id] = CudaUtils::FaceNormal(posV1, posV2, posV3);


	float3 min = CudaUtils::min3(posV1, posV2, posV3);
	float3 max = CudaUtils::max3(posV1, posV2, posV3);

	aabbMins[id] = min;
	aabbMaxs[id] = max;

	mortonCodes[id] = CudaUtils::Morton3D(min + (max - min) / 2.f);
}


__global__ void AdjustSprings(const int particleCount, float3 *positions, const int springCount, int *aIDs, int *bIDs, float *l0s, const float maxDeformation, bool *fixedVerts)
{
	int id = CudaUtils::MyID();

	if (id >= springCount)
		return;

	int aID = aIDs[id], bID = bIDs[id];
	float l0 = l0s[id];

	float deformation = ComputeSpringDeformation(positions[aID], positions[bID], l0);

	if (deformation > maxDeformation)
	{
		bool aFixed = fixedVerts[aID], bFixed = fixedVerts[bID];

		if (aFixed && bFixed)
			return;

		float dif = (deformation - maxDeformation) * l0;
		float3 push = CudaUtils::normalize(positions[bID] - positions[aID]) * dif;

		if (!aFixed && bFixed)
		{
			atomicAdd(&(positions[aIDs[id]].x), push.x / 2);
			atomicAdd(&(positions[aIDs[id]].y), push.y / 2);
			atomicAdd(&(positions[aIDs[id]].z), push.z / 2);

			//positions[aIDs[id]] = positions[aIDs[id]] + push / 2;
		}
		else if (aFixed && !bFixed)
		{
			//positions[bIDs[id]] = positions[bIDs[id]] - push / 2;

			atomicAdd(&(positions[bIDs[id]].x), -push.x / 2);
			atomicAdd(&(positions[bIDs[id]].y), -push.y / 2);
			atomicAdd(&(positions[bIDs[id]].z), -push.z / 2);
		}
		else
		{
			atomicAdd(&(positions[aIDs[id]].x), push.x / 4);
			atomicAdd(&(positions[aIDs[id]].y), push.y / 4);
			atomicAdd(&(positions[aIDs[id]].z), push.z / 4);

			atomicAdd(&(positions[bIDs[id]].x), -push.x / 4);
			atomicAdd(&(positions[bIDs[id]].y), -push.y / 4);
			atomicAdd(&(positions[bIDs[id]].z), -push.z / 4);

			//positions[aIDs[id]] = positions[aIDs[id]] + push / 4;
			//positions[bIDs[id]] = positions[bIDs[id]] - push / 4;
		}
	}

}

__global__ void AdjustSprings(const int particleCount, float3 *positions, int *bIDs, float *l0s, const float maxDeformation, int *springInfo, int *springIDs, bool *fixedVerts)
{
	int id = CudaUtils::MyID();

	if (id >= particleCount || fixedVerts[id])
		return;

	float3 correction = make_float3(0.f, 0.f, 0.f);

	for (int i = springIDs[id]; i < springIDs[id + 1]; ++i)
	{
		int sprID = springInfo[i];

		float deformation = ComputeSpringDeformation(positions[id], positions[bIDs[sprID]], l0s[sprID]);


		if (deformation > maxDeformation)
		{
			//printf("[%d] Deformation = %f\n", id, deformation);

			//float dif = (deformation - maxDeformation) * l0s[sprID];
			float dif = CudaUtils::distance(positions[id], positions[bIDs[sprID]]) - l0s[sprID];

			float3 push = CudaUtils::normalize(positions[bIDs[sprID]] - positions[id]) * dif / (fixedVerts[bIDs[sprID]] ? 1.f : 2.f);

			//printf("[%d] Push = (%f, %f, %f)\n", id, push.x, push.y, push.z);
			printf("[%d] dif = %f\n", id, dif);


			correction = correction + push;
		}
	}

	__syncthreads();
	positions[id] = positions[id] + correction;
}

void Physics::CudaDeformableBody::FixedUpdate()
{

	int numBlocks = (m_particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	auto start = TimeUtils::Now();
	ClothEngineStep<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(m_particleCount, CudaUtils::ToRaw(m_dPositions), CudaUtils::ToRaw(m_dPrevPositions), CudaUtils::ToRaw(m_dVelocities),
		CudaUtils::ToRaw(m_dMasses), m_springCount, CudaUtils::ToRaw(m_daIDs), CudaUtils::ToRaw(m_dbIDs), CudaUtils::ToRaw(m_dks), CudaUtils::ToRaw(m_dl0s),
		CudaUtils::ToRaw(m_dLinSpringInfo),	CudaUtils::ToRaw(m_dSpringIDs), CudaUtils::CUDA_GRAVITY_ACCEL, m_dampingCoefficient, Core::PHYSICS_TIME_STEP, m_bendStiffness, CudaUtils::ToRaw(m_fixedVerts));

	auto end = TimeUtils::Now();

	std::cout << "Cloth Engine step: " << TimeUtils::NToMs(TimeUtils::DurationNano(start, end)) << "ms" << std::endl;
	cudaCheckError();

	numBlocks = (m_triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	start = TimeUtils::Now();
	UpdateTriangles << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_triangleCount, CudaUtils::ToRaw(m_dPositions), CudaUtils::ToRaw(m_dTriV1s),
		CudaUtils::ToRaw(m_dTriV2s), CudaUtils::ToRaw(m_dTriV3s), CudaUtils::ToRaw(m_dFaceNormals), CudaUtils::ToRaw(m_dMortonCodes),
		CudaUtils::ToRaw(m_dAABBMins), CudaUtils::ToRaw(m_dAABBMaxs));

	end = TimeUtils::Now();

	std::cout << "Triangle update step: " << TimeUtils::NToMs(TimeUtils::DurationNano(start, end)) << "ms" << std::endl;

	start = TimeUtils::Now();
	thrust::sort_by_key(thrust::device, m_dMortonCodes.begin(), m_dMortonCodes.end(), m_dTriangleIDs.begin(), thrust::less<unsigned int>());	
	end = TimeUtils::Now();

	std::cout << "Morton sort: " << TimeUtils::NToMs(TimeUtils::DurationNano(start, end)) << "ms" <<std::endl;

	//AdjustSprings << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_particleCount, CudaUtils::ToRaw(m_dPositions), CudaUtils::ToRaw(m_dbIDs), CudaUtils::ToRaw(m_dl0s),
	//	0.1f, CudaUtils::ToRaw(m_dLinSpringInfo), CudaUtils::ToRaw(m_dSpringIDs), CudaUtils::ToRaw(m_fixedVerts));

	//numBlocks = (m_springCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	/*AdjustSprings<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(m_particleCount, CudaUtils::ToRaw(m_dPositions), m_springCount,
		CudaUtils::ToRaw(m_daIDs), CudaUtils::ToRaw(m_dbIDs), CudaUtils::ToRaw(m_dl0s), 0.1f, CudaUtils::ToRaw(m_fixedVerts));*/



	start = TimeUtils::Now();
	thrust::host_vector<float3> finalPos = m_dPositions;
	for (int i = 0; i < m_verts->size(); ++i)
	{
		(*m_verts)[i].m_position = glm::vec3(finalPos[i].x, finalPos[i].y, finalPos[i].z);
	}

	end = TimeUtils::Now();

	std::cout << "CPU Vertex position update: " << TimeUtils::NToMs(TimeUtils::DurationNano(start, end)) << "ms" << std::endl;

	std::cout << "---------------------------------" << std::endl;
}

void Physics::CudaDeformableBody::Update()
{

}
