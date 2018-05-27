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
#include "CudaPBD.cuh"


#define _PRINT_TIMERS

int Physics::CudaDeformableBody::lin(const int i, const int j)
{
	return i * m_dims.first + j;
}

void Physics::CudaDeformableBody::ClothInternalDynamics()
{
#ifdef _PRINT_TIMERS
	
	m_timer.Start();
#endif // _PRINT_TIMERS


	CudaMassSpring::ClothEngineStep(m_dPositions, m_dPrevPositions, m_dVelocities, m_dMasses, m_daIDs, m_dbIDs, m_dks, m_dl0s, m_dLinSpringInfo, m_dSpringIDs,
		CudaUtils::CUDA_GRAVITY_ACCEL, m_dampingCoefficient, m_springDampingCoefficient, Core::PHYSICS_TIME_STEP, m_bendStiffness, m_fixedVerts);


	CudaPBD::DampVelocities(m_dPositions, m_dMasses, m_dampingCoefficient, m_dVelocities, m_pbdAux1, m_pbdAux2, m_dTempStorage,
		m_dTempStorageSize);

#ifdef _PRINT_TIMERS
	std::cout << "Cloth engine step: " << m_timer.End() << std::endl;

	m_timer.Start();
#endif // _PRINT_TIMERS

	CudaMassSpring::AdjustSprings(m_dPositions, m_dPrevPositions, m_dVelocities,
		m_dbIDs, m_dl0s, 0.1f, m_dLinSpringInfo, m_dSpringIDs, m_fixedVerts,
		m_dks, m_bendStiffness, Core::PHYSICS_TIME_STEP);



#ifdef _PRINT_TIMERS

	std::cout << "Adjust springs: " << m_timer.End() << std::endl;
#endif
}

void Physics::CudaDeformableBody::UpdateTrianglesDiscrete()
{
#ifdef _PRINT_TIMERS
	m_timer.Start();
#endif
	DeformableUtils::UpdateTriangles(m_dPositions, m_dTriangles, m_dMortonCodes, m_dAABBMins, m_dAABBMaxs, m_thickness);
	DeformableUtils::UpdateVertexNormals(m_dTriangles, m_dRawVertexNormals, m_dRawVertexNormalIDs, m_dAccumulatedVertexNormals);

#ifdef _PRINT_TIMERS
	std::cout << "Update Triangles Discrete: " << m_timer.End() << std::endl;
#endif
}

void Physics::CudaDeformableBody::UpdateTrianglesContinuous()
{

}

void Physics::CudaDeformableBody::BuildBVH()
{
#ifdef _PRINT_TIMERS
	m_timer.Start();
#endif

	DeformableUtils::SortMortons(m_dMortonCodes, m_dTriangles, m_dTempStorage, m_dTempStorageSize);

#ifdef _PRINT_TIMERS
	std::cout << "Morton sort: " << m_timer.End() << std::endl;


	m_timer.Start();
#endif

	CudaBVH::GenerateBVH2(m_dMortonCodes, m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dNodesVisited);

#ifdef _PRINT_TIMERS
	std::cout << "Radix-tree build: " << m_timer.End() << std::endl;


	m_timer.Start();
#endif

	CudaBVH::ComputeTreeAABBs(m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dNodesVisited, m_dAABBMins, m_dAABBMaxs, m_dRightMostLeafLefts, m_dRightMostLeafRights);

	//CudaBVH::BVHTestUnit(m_dMortonCodes, m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dNodesVisited, m_dAABBMins, m_dAABBMaxs);

#ifdef _PRINT_TIMERS
	std::cout << "BVH AABBs: " << m_timer.End() << std::endl;


	//CudaBVH::PrintTree(m_dTreeLefts, m_dTreeRights, m_dTreeParents, m_dRightMostLeafLefts, m_dRightMostLeafRights);
#endif
}

void Physics::CudaDeformableBody::HandleCollisionsDiscrete()
{
#ifdef _PRINT_TIMERS
	m_timer.Start();
#endif

	CudaBVH::GetAABBCollisions(m_dTreeLefts, m_dTreeRights, m_dAABBMins, m_dAABBMaxs, m_dRightMostLeafLefts,
		m_dRightMostLeafRights, m_dAABBCollisions, m_AABBColChunkSize, m_timeStamp);

#ifdef _PRINT_TIMERS
	std::cout << "AABB collisions: " << m_timer.End() << std::endl;

	m_timer.Start();
#endif

	DeformableUtils::CreateTriangleTests(m_dTriangles, m_dAABBCollisions, m_timeStamp,
		m_vfContacts, m_vfContactsSize, m_eeContacts, m_eeContactsSize, m_dTempStorage, m_dTempStorageSize);

#ifdef _PRINT_TIMERS
	std::cout << "Primitive tests creation: " << m_timer.End() << std::endl;

	m_timer.Start();
#endif

	DeformableUtils::DCDTriangleTests(m_dTriangles, m_dPositions, m_vfContacts, m_vfContactsSize,
		m_eeContacts, m_eeContactsSize, m_thickness, m_dTempStorage, m_dTempStorageSize);


#ifdef _PRINT_TIMERS
	std::cout << "DCD primitive tests: " << m_timer.End() << std::endl;
#endif


	DeformableUtils::ColorCollidingFeatures(m_dVerts, m_vfContacts, m_vfContactsSize, m_eeContacts, m_eeContactsSize);

#ifdef _PRINT_TIMERS
	m_timer.Start();
#endif

	/*DeformableUtils::CreateImpulses(m_dPositions, m_dVelocities, m_vfContacts, m_vfContactsSize, m_eeContacts, m_eeContactsSize,
	m_dImpulseIDs, m_dImpulseValues, m_impulsesSize, m_dImpulseRLEUniques, m_dImpulseRLECounts, m_impulseRunCount,
	m_dAccumulatedImpulses, m_structuralStiffness, m_vertexMass, Core::PHYSICS_TIME_STEP, m_thickness,
	m_dTempStorage, m_dTempStorageSize);*/


	DeformableUtils::CreateImpulses(m_dPositions, m_dVelocities, m_vfContacts, m_vfContactsSize, m_eeContacts, m_eeContactsSize,
		m_dbImpulseID, m_dbImpulseValues, m_impulsesSize, m_dImpulseRLEUniques, m_dImpulseRLECounts, m_impulseRunCount,
		m_dAccumulatedImpulses, m_structuralStiffness, m_vertexMass, Core::PHYSICS_TIME_STEP, m_thickness,
		m_dTempStorage, m_dTempStorageSize);

#ifdef _PRINT_TIMERS
	std::cout << "DCD Impulse creation: " << m_timer.End() << std::endl;
#endif

#ifdef _PRINT_TIMERS
	m_timer.Start();
#endif

	DeformableUtils::ApplyImpulses(m_dPositions, m_dPrevPositions, m_dVelocities,
		m_dAccumulatedImpulses, m_fixedVerts, m_vertexMass, Core::PHYSICS_TIME_STEP);

#ifdef _PRINT_TIMERS
	std::cout << "DCD Impulse application: " << m_timer.End() << std::endl;
#endif
}

void Physics::CudaDeformableBody::HandleCollisionsContinuous()
{

}

void Physics::CudaDeformableBody::FinalVertUpdate()
{

#ifdef _PRINT_TIMERS
	m_timer.Start();
#endif

	DeformableUtils::FinalVerticesUpdate(m_dVerts, m_dTriangles, m_dPositions, m_dAccumulatedVertexNormals);

	cudaMemcpy(m_verts->data(), thrust::raw_pointer_cast(&m_dVerts[0]), m_dVerts.size() * sizeof(Rendering::VertexFormat), cudaMemcpyDeviceToHost);

#ifdef _PRINT_TIMERS
	std::cout << "Final vert update: " << m_timer.End() << std::endl;
#endif

}

Physics::CudaDeformableBody::~CudaDeformableBody()
{

}

Physics::CudaDeformableBody::CudaDeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices, std::pair<int, int> dims)
{

	m_freeVRAMInit = CudaUtils::VRAMUsage();

	m_structuralStiffness = 0.8f;
	m_bendStiffness = 0.01f;
	m_shearStiffness = 0.4f;
	m_dampingCoefficient = 0.3f;
	m_springDampingCoefficient = 0.2f;

	m_objectMass = 1.f;

	m_verts = verts;
	m_indices = indices;
	m_dims = dims;

	m_vertexMass = m_objectMass / (m_dims.first * m_dims.second);

	thrust::host_vector<float3> initPos(m_verts->size()), initVel(m_verts->size());
	thrust::host_vector<unsigned int> cudaIndices(indices->size());
	thrust::host_vector<float> masses(m_verts->size());

	thrust::host_vector<Rendering::VertexFormat> hVerts(m_verts->size());

	for (int i = 0; i < cudaIndices.size(); ++i)
	{
		cudaIndices[i] = (*indices)[i];
	}

	m_particleCount = initPos.size();
	thrust::host_vector<bool> hfVerts(m_particleCount);


	for (int i = 0; i < dims.first; ++i)
	{
		hfVerts[i * dims.first] = true;
		//hfVerts[(dims.first - 1)* dims.first ] = true;
		break;
	}


	for (int i = 0; i < m_verts->size(); ++i)
	{
		auto vert = (*m_verts)[i];

		initPos[i] = make_float3(vert.m_position.x, vert.m_position.y, vert.m_position.z);
		initVel[i] = make_float3(Core::Utils::RandomAround(0.f, 0.2f), Core::Utils::RandomAround(0.f, 0.2f), Core::Utils::RandomAround(0.f, 20.f));
		hVerts[i] = vert;
		//initVel.push_back(make_float3(0.f, 0.f, 0.f));
		//masses.push_back(1.0f);
		masses[i] = hfVerts[i] ? 1000000.f : m_vertexMass;
	}

	m_fixedVerts = hfVerts;
	hfVerts.clear();

	m_thickness = glm::distance(CudaUtils::MakeVec(initPos[0]), CudaUtils::MakeVec(initPos[1])) / 10.f;

	//thrust::host_vector<Rendering::VertexFormat> hVerts = *verts;
	//m_dVerts.resize(m_verts->size());
	//thrust::copy(m_verts->begin(), m_verts->end(), m_dVerts);

	m_dVerts = hVerts;
	hVerts.clear();
	



	//m_dVertexNormalCounts.resize(initPos.size());

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


	m_dAccumulatedImpulses.resize(m_particleCount);
	m_impulsesSize = m_particleCount * 4;

	//m_dImpulseIDs.resize(m_impulsesSize);
	//m_dAltImpulseIDs.resize(m_impulsesSize);

	m_dbImpulseID.buffers[0].resize(m_impulsesSize);
	m_dbImpulseID.buffers[1].resize(m_impulsesSize);

	m_dbImpulseValues.buffers[0].resize(m_impulsesSize);
	m_dbImpulseValues.buffers[1].resize(m_impulsesSize);

	//m_dImpulseValues.resize(m_impulsesSize);
	//m_dAltImpulseValues.resize(m_impulsesSize);

	m_dImpulseRLEUniques.resize(m_impulsesSize);
	m_dImpulseRLECounts.resize(m_impulsesSize);



	m_dRawVertexNormals.resize(m_triangleCount * 3);
	m_dRawVertexNormalIDs.resize(m_triangleCount * 3);
	m_dAccumulatedVertexNormals.resize(m_particleCount);


	m_pbdAux1.resize(m_particleCount);
	m_pbdAux2.resize(m_particleCount);

	m_timeStamp = 1;
	cudaCheckError();

	std::cout << "Particle count: " << m_particleCount << ", triangle count: " << m_triangleCount << std::endl;

	std::cout << "Init over. " << CudaUtils::MemUsage(m_freeVRAMInit) << std::endl;

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
	ClothInternalDynamics();

	UpdateTrianglesDiscrete();

	BuildBVH();	
	
	HandleCollisionsDiscrete();

	HandleCollisionsContinuous();


	DeformableUtils::WorldConstraints(m_dPositions);




#ifdef _PRINT_TIMERS
	std::cout << "FixedUpdate over. " << CudaUtils::MemUsage(m_freeVRAMInit) << std::endl;
	std::cout << "---------------------------------" << std::endl;
#endif


	m_timeStamp++;
}

void Physics::CudaDeformableBody::Update()
{
	FinalVertUpdate();
}
