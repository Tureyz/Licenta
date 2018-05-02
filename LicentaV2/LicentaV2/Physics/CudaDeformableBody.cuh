#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include "IPhysicsbody.h"

struct CudaSpring
{
	int m_aID;
	int m_bID;
	float m_stiffness;
	float m_initialLength;
};

namespace Physics
{

	struct SpringInfo
	{
		int *m_springIds;
		int m_count;
	};


	struct ParticleInfoList
	{
		float3 *m_pos;
		float3 *m_prevPos;
		float3 *m_vel;
		float *m_mass;
		int m_count;
	};

	struct SpringInfoList
	{
		int *m_a;
		int *m_b;
		int *m_springInfo;
		int *m_springIDs;

		float *m_k;
		float *m_l0;

		int m_count;
	};

	struct SimulationInfo
	{
		float3 m_gravity;
		float m_damping;
		float m_timeStep;
		float m_bend;
	};


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

		/*thrust::host_vector<float3> m_vertexPositions;
		thrust::host_vector<float3> m_vertexVelocities;
		thrust::host_vector<float> m_particleMasses;
		thrust::host_vector<CudaSpring> m_springs;

		thrust::host_vector<thrust::host_vector<int>> m_springInfo;
		thrust::host_vector<int> m_springIDs;*/


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
		thrust::device_vector<int> m_dTriangleIDs;
		thrust::device_vector<int> m_dTriV1s;
		thrust::device_vector<int> m_dTriV2s;
		thrust::device_vector<int> m_dTriV3s;
		thrust::device_vector<float3> m_dFaceNormals;
		thrust::device_vector<unsigned int> m_dMortonCodes;
		thrust::device_vector<float3> m_dAABBMins;
		thrust::device_vector<float3> m_dAABBMaxs;		


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
