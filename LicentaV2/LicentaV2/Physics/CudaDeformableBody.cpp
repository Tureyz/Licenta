#include "CudaDeformableBody.h"

#include <iostream>
#include <algorithm>

#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/host_vector.h>

#include "../Core/Utils.hpp"
#include "../Core/TimeUtils.h"
#include "../Core/CudaUtils.cuh"

#include "../Collision/CudaBVH.cuh"
#include "../Collision/CudaPrimitiveTests.cuh"

#include "CudaDeformableUtils.cuh"
#include "CudaMassSpring.cuh"



int Physics::CudaDeformableBody::lin(const int i, const int j)
{
	return i * m_dims.first + j;
}

Physics::CudaDeformableBody::~CudaDeformableBody()
{

}

Physics::CudaDeformableBody::CudaDeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices, std::pair<int, int> dims)
{

	m_structuralStiffness = 0.9f;
	m_bendStiffness = 0.3f;
	m_shearStiffness = 0.7f;
	m_dampingCoefficient = 0.3f;

	m_objectMass = 1.f;

	m_verts = verts;
	m_indices = indices;
	m_dims = dims;

	float vertexMass = m_objectMass / (m_dims.first * m_dims.second);

	thrust::host_vector<float3> initPos(m_verts->size()), initVel(m_verts->size());
	thrust::host_vector<unsigned int> cudaIndices(indices->size());
	thrust::host_vector<float> masses(m_verts->size());

	thrust::host_vector<Rendering::VertexFormat> hVerts(m_verts->size());

	for (int i = 0; i < cudaIndices.size(); ++i)
	{
		cudaIndices[i] = (*indices)[i];
	}

	for (int i = 0; i < m_verts->size(); ++i)
	{
		auto vert = (*m_verts)[i];

		initPos[i] = make_float3(vert.m_position.x, vert.m_position.y, vert.m_position.z);
		initVel[i] = make_float3(Core::Utils::RandomAround(0.f, 0.2f), Core::Utils::RandomAround(0.f, 0.2f), Core::Utils::RandomAround(0.f, 20.f));
		hVerts[i] = vert;
		//initVel.push_back(make_float3(0.f, 0.f, 0.f));
		//masses.push_back(1.0f);
		masses[i] = vertexMass;
	}

	m_thickness = glm::distance(CudaUtils::MakeVec(initPos[0]), CudaUtils::MakeVec(initPos[1]));

	//thrust::host_vector<Rendering::VertexFormat> hVerts = *verts;
	//m_dVerts.resize(m_verts->size());
	//thrust::copy(m_verts->begin(), m_verts->end(), m_dVerts);

	std::cout << sizeof(Rendering::VertexFormat) * hVerts.size() << std::endl;
	m_dVerts = hVerts;
	hVerts.clear();
	

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

// 			if (i + 2 < m_dims.first && j + 2 < m_dims.first)
// 				AddSpring(i, j, i + 2, j + 2, m_bendStiffness, aIDs, bIDs, ks, l0s, springInfo, initPos);
		}
	}

	thrust::host_vector<int> springIDs(springInfo.size() + 1);
	springIDs[0] = 0;

	for (int i = 1; i < springInfo.size() + 1; ++i)
	{
		springIDs[i] = springIDs[i - 1] + springInfo[i - 1].size();
	}


	thrust::host_vector<int> linSpringInfo;

	for (auto vec : springInfo)
	{
		linSpringInfo.insert(linSpringInfo.end(), vec.begin(), vec.end());
	}



	//for (auto springID : springIDs)
	//{
	//	std::cout << springID << std::endl;
	//}

	//std::cout << "SpringInfoSize: " << linSpringInfo.size() << ", last springID: " << springIDs.back() << std::endl;

	m_dPrevPositions = initPos;
	m_dPositions = initPos;
	m_particleCount = initPos.size();	
	m_dForces.resize(initPos.size(), make_float3(0, 0, 0));

	initPos.clear();

	m_dVelocities = initVel;
	m_dMasses = masses;
	m_dSpringIDs = springIDs;
	m_dLinSpringInfo = linSpringInfo;

	m_daIDs = aIDs;
	m_dbIDs = bIDs;
	m_dks = ks;
	m_dl0s = l0s;

	m_springCount = aIDs.size();

	m_fixedVerts.resize(m_particleCount);

	for (int i = 0; i < dims.first; ++i)
	{
		m_fixedVerts[i * dims.first] = true;
		break;
	}


	m_triangleCount = cudaIndices.size() / 3;

	/*m_dTriangleIDs.resize(m_triangleCount);
	thrust::sequence(m_dTriangleIDs.begin(), m_dTriangleIDs.end());
	m_dTriV1s.resize(m_triangleCount);
	m_dTriV2s.resize(m_triangleCount);
	m_dTriV3s.resize(m_triangleCount);
	m_dFaceNormals.resize(m_triangleCount);*/

	m_dTriangles.resize(m_triangleCount);
	m_dMortonCodes.resize(m_triangleCount);
	m_dTempStorage = NULL;
	m_dTempStorageSize = 0;


	m_dAABBMins.resize(2 * m_triangleCount - 1);
	m_dAABBMaxs.resize(2 * m_triangleCount - 1);


	thrust::device_vector<unsigned int> dCudaIndices = cudaIndices;
	cudaIndices.clear();


	//int numBlocks = (m_triangleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;
	

	DeformableUtils::CreateTriangles(m_dPositions, dCudaIndices, m_dTriangles, m_dMortonCodes, m_dAABBMins, m_dAABBMaxs, m_thickness);

	/*CreateTriangles<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(m_triangleCount, CudaUtils::ToRaw(m_dPositions), CudaUtils::ToRaw(dCudaIndices), CudaUtils::ToRaw(m_dTriV1s),
		CudaUtils::ToRaw(m_dTriV2s), CudaUtils::ToRaw(m_dTriV3s), CudaUtils::ToRaw(m_dFaceNormals), CudaUtils::ToRaw(m_dMortonCodes), CudaUtils::ToRaw(m_dAABBMins),
		CudaUtils::ToRaw(m_dAABBMaxs));*/



	m_internalNodeCount = m_triangleCount - 1;
	m_dTreeLefts.resize(m_internalNodeCount);
	m_dTreeRights.resize(m_internalNodeCount);
	m_dTreeParents.resize(m_triangleCount + m_internalNodeCount);
	m_dNodesVisited.resize(m_internalNodeCount);

	m_dRightMostLeafLefts.resize(m_internalNodeCount);
	m_dRightMostLeafRights.resize(m_internalNodeCount);


	m_AABBColChunkSize = 256 < m_triangleCount ? 256 : m_triangleCount;

	m_dAABBCollisions.resize(m_triangleCount * m_AABBColChunkSize);
	//m_dFilteredAABBCollisions.resize(m_triangleCount * m_AABBColChunkSize);
	//m_dAABBCollisionSizes.resize(m_triangleCount); // + 1 for prefix sums

	m_timeStamp = 1;
	cudaCheckError();

	std::cout << "Init over. " << CudaUtils::MemUsage() << std::endl;

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





void Physics::CudaDeformableBody::FixedUpdate()
{
	CudaUtils::CudaTimer timer;

	
	timer.Start();

	CudaMassSpring::ClothEngineStep(m_dPositions, m_dPrevPositions, m_dVelocities, m_dMasses, m_daIDs, m_dbIDs, m_dks, m_dl0s, m_dLinSpringInfo, m_dSpringIDs,
		CudaUtils::CUDA_GRAVITY_ACCEL, m_dampingCoefficient, Core::PHYSICS_TIME_STEP, m_bendStiffness, m_fixedVerts);

	std::cout << "Cloth engine step: " << timer.End() << std::endl;

	timer.Start();

	CudaMassSpring::AdjustSprings(m_dPositions, m_dbIDs, m_dl0s, 0.1f, m_dLinSpringInfo, m_dSpringIDs, m_fixedVerts,
		m_dks, m_bendStiffness);
	

	std::cout << "Adjust springs: " << timer.End() << std::endl;	


	timer.Start();
	
	DeformableUtils::UpdateTriangles(m_dPositions, m_dTriangles, m_dMortonCodes, m_dAABBMins, m_dAABBMaxs, m_thickness);

	std::cout << "Update Triangles: " << timer.End() << std::endl;

	
	//DeformableUtils::AddWind(m_dTriangles, m_dPositions, m_dMasses, Core::PHYSICS_TIME_STEP);

	
	
	timer.Start();
	DeformableUtils::SortMortons(m_dMortonCodes, m_dTriangles, m_dTempStorage, m_dTempStorageSize);
	std::cout << "Morton sort: " << timer.End() << std::endl;


	timer.Start();
	CudaBVH::GenerateBVH2(m_dMortonCodes, m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dNodesVisited);
	std::cout << "Radix-tree build: " << timer.End() << std::endl;


	timer.Start();
	CudaBVH::ComputeTreeAABBs(m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dNodesVisited, m_dAABBMins, m_dAABBMaxs, m_dRightMostLeafLefts, m_dRightMostLeafRights);
	std::cout << "BVH AABBs: " << timer.End() << std::endl;

	CudaBVH::BVHTestUnit(m_dMortonCodes, m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dNodesVisited, m_dAABBMins, m_dAABBMaxs);
	
	//CudaBVH::PrintTree(m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dRightMostLeafLefts, m_dRightMostLeafRights);

	
	timer.Start();
	CudaBVH::GetAABBCollisions(m_dTreeLefts, m_dTreeRights, m_dAABBMins, m_dAABBMaxs, m_dRightMostLeafLefts,
		m_dRightMostLeafRights, m_dAABBCollisions, m_AABBColChunkSize, m_timeStamp);

	std::cout << "AABB collisions: " << timer.End() << std::endl;

	timer.Start();

	DeformableUtils::CreateTriangleTests(m_dTriangles, m_dAABBCollisions, m_timeStamp,
		m_vfContacts, m_vfContactsSize, m_eeContacts, m_eeContactsSize, m_dTempStorage, m_dTempStorageSize);

	std::cout << "Primitive tests creation: " << timer.End() << std::endl;

	timer.Start();
	CudaPrimitiveTests::DCDTriangleTests(m_dTriangles, m_dPositions, m_vfContacts, m_vfContactsSize, m_eeContacts, m_eeContactsSize, m_thickness);
	std::cout << "DCD primitive tests: " << timer.End() << std::endl;

	DeformableUtils::WorldConstraints(m_dPositions);

	
	timer.Start();
	DeformableUtils::FinalVerticesUpdate(m_dVerts, m_dTriangles, m_dPositions);

	cudaMemcpy(m_verts->data(), thrust::raw_pointer_cast(&m_dVerts[0]), m_dVerts.size() * sizeof(Rendering::VertexFormat), cudaMemcpyDeviceToHost);	

	std::cout << "Final vert update: " << timer.End() << std::endl;
	std::cout << "FixedUpdate over. " << CudaUtils::MemUsage() << std::endl;
	std::cout << "---------------------------------" << std::endl;


	m_timeStamp++;
}

void Physics::CudaDeformableBody::Update()
{

}
