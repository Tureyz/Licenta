#pragma once

#include <thrust/device_vector.h>
#include "IPhysicsbody.h"

#include "Structs.h"


namespace Physics
{

	

	class CudaDeformableBody : public IPhysicsbody
	{
	public:

		~CudaDeformableBody();
		CudaDeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices, std::pair<int, int> dims);
		virtual void FixedUpdate() override;
		virtual void Update() override;



	private:

		void AddSpring(const int i1, const int j1, const int i2, const int j2, const float stiffness,
			thrust::host_vector<int> &aIDs, thrust::host_vector<int> &bIDs, thrust::host_vector<float> &ks,
			thrust::host_vector<float> &l0s, thrust::host_vector<thrust::host_vector<int>> &springInfo,
			const thrust::host_vector<float3> &initPos);
		int lin(const int i, const int j);
		//__device__ int MyID();
		////__global__ void CudaFixedUpdate(float3 *d_positions, int posSize, float3 *d_velocities, int velSize, float *masses, int massesSize, struct CudaSpring *springs, int springsSize);
		//__global__ void ComputeExternalForces(float3 *forces, int forcesSize, float *masses, int massesSize);

		//__global__ void ApplyForces(float3 *forces, int forcesSize, float *masses, int massesSize, float3 *d_positions, int posSize, float3 *d_velocities, int velSize);

		std::pair<int, int> m_dims;

		float m_thickness;

		/*thrust::host_vector<float3> m_vertexPositions;
		thrust::host_vector<float3> m_vertexVelocities;
		thrust::host_vector<float> m_particleMasses;
		thrust::host_vector<CudaSpring> m_springs;

		thrust::host_vector<thrust::host_vector<int>> m_springInfo;
		thrust::host_vector<int> m_springIDs;*/


		thrust::device_vector<Rendering::VertexFormat> m_dVerts;

		int m_particleCount;
		thrust::device_vector<bool> m_fixedVerts;
		thrust::device_vector<float3> m_dPrevPositions;
		thrust::device_vector<float3> m_dPositions;
		thrust::device_vector<float3> m_dVelocities;
		thrust::device_vector<float> m_dMasses;
		thrust::device_vector<float3> m_dForces;
		thrust::device_vector<float3> m_dVertexNormals;


		int m_springCount;
		thrust::device_vector<int> m_daIDs;
		thrust::device_vector<int> m_dbIDs;
		thrust::device_vector<float> m_dks;
		thrust::device_vector<float> m_dl0s;
		thrust::device_vector<int> m_dSpringIDs;
		thrust::device_vector<int> m_dLinSpringInfo;


		int m_triangleCount;
		//thrust::device_vector<int> m_dTriangleIDs;
		//thrust::device_vector<int> m_dTriV1s;
		//thrust::device_vector<int> m_dTriV2s;
		//thrust::device_vector<int> m_dTriV3s;
		//thrust::device_vector<float3> m_dFaceNormals;


		thrust::device_vector<CudaTriangle> m_dTriangles;
		thrust::device_vector<uint64_t> m_dMortonCodes;
		thrust::device_vector<float3> m_dAABBMins;
		thrust::device_vector<float3> m_dAABBMaxs;
		void *m_dTempStorage;
		uint64_t m_dTempStorageSize;

		int m_internalNodeCount;
		thrust::device_vector<int> m_dNodesVisited;
		thrust::device_vector<int> m_dTreeLefts;
		thrust::device_vector<int> m_dTreeRights;
		thrust::device_vector<int> m_dTreeParents;
		thrust::device_vector<int> m_dRightMostLeafLefts;
		thrust::device_vector<int> m_dRightMostLeafRights;
		//thrust::device_vector<float3> m_dTreeAABBMins;
		//thrust::device_vector<float3> m_dTreeAABBMaxs;
		//thrust::device_vector<int> m_dTreeBeginIDs;
		//thrust::device_vector<int> m_dTreeEndIDs;

		thrust::device_vector<Physics::AABBCollision> m_dAABBCollisions;
		thrust::device_vector<Physics::AABBCollision> m_dFilteredAABBCollisions;
		uint64_t m_filteredAABBCollisionsSize;
		
		//thrust::device_vector<int> m_dAABBCollisionSizes;

		thrust::device_vector<Physics::PrimitiveContact> m_vfContacts;
		uint64_t m_vfContactsSize;
		thrust::device_vector<Physics::PrimitiveContact> m_eeContacts;
		uint64_t m_eeContactsSize;


		uint64_t m_timeStamp;
		int m_AABBColChunkSize;

		ParticleInfoList m_particleInfos;
		SpringInfoList m_springInfos;
		SimulationInfo m_simulationInfo;


		/*float3 *m_dPos;
		float3 *m_dVel;
		float *m_dMasses;
		float3 *m_dForces;

		int *m_daids;
		int *m_dbids;
		float *m_dks;
		float *m_dl0s;


		int *m_dSpringIDs;
		int *m_dLinSpringInfo;*/

		/*int **m_springInfo;
		int *m_springIDs;*/


		float m_structuralStiffness;
		float m_shearStiffness;
		float m_bendStiffness;
		float m_dampingCoefficient;
		float m_objectMass;
	};

}
