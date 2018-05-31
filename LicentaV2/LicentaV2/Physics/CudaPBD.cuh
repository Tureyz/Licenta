#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/device_vector.h>

#include "Structs.h"
#include "../Rendering/VertexFormat.h"

namespace CudaPBD
{

	struct StretchConstraintList
	{
		thrust::device_vector<int> id1;
		thrust::device_vector<int> id2;

		thrust::device_vector<int> id1Offset;
		thrust::device_vector<int> id2Offset;

		//thrust::device_vector<float> k;
		float k;
		thrust::device_vector<float> l0;
	};

	struct BendConstraintList
	{
		thrust::device_vector<int> id1;
		thrust::device_vector<int> id2;
		thrust::device_vector<int> id3;
		thrust::device_vector<int> id4;

		thrust::device_vector<int> id1Offset;
		thrust::device_vector<int> id2Offset;
		thrust::device_vector<int> id3Offset;
		thrust::device_vector<int> id4Offset;

		float k;
		//thrust::device_vector<float> k;
		thrust::device_vector<float> phi0;
	};


	struct CudaPBD
	{
		Physics::ClothParams m_parentParams;

		StretchConstraintList m_stretchConstraints;
		BendConstraintList m_bendConstraints;


		//thrust::device_vector<int> m_staticCorrectionIDs;
		thrust::device_vector<float3> m_staticCorrectionValues;
		thrust::device_vector<int> m_staticCorrectionPrefixSums;
		thrust::device_vector<float3> m_accumulatedStaticCorrectionValues;

		thrust::device_vector<float3> m_aux1;
		thrust::device_vector<float3> m_aux2;


		void PBDStepExternal(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions, const thrust::device_vector<float> &masses,
			const thrust::device_vector<float> &invMasses, const thrust::device_vector<bool> &fixedVerts,
			thrust::device_vector<float3> &velocities, void *&tempStorage, uint64_t &tempStorageSize);

		

		/*void PBDFinalUpdate(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions, 
			thrust::device_vector<float3> &velocities, const thrust::device_vector<bool> &fixedVerts);*/

		void Init(const Physics::ClothParams parentParams, const std::vector<Rendering::VertexFormat> &verts, const std::vector<unsigned int> &indices);

		void CreateStaticStretchConstraint(const int id1, const int id2, const std::vector<Rendering::VertexFormat> &verts,
			thrust::host_vector<int> &sid1, thrust::host_vector<int> &sid2, thrust::host_vector<float> &sl0);

		void CreateStaticBendConstraint(const int3 tri1, const int3 tri2, const std::vector<Rendering::VertexFormat> &verts,
			thrust::host_vector<int> &bid1, thrust::host_vector<int> &bid2, thrust::host_vector<int> &bid3,
			thrust::host_vector<int> &bid4,
			thrust::host_vector<float> &bphi0);

		void DampVelocities(const thrust::device_vector<float3> &positions, const thrust::device_vector<float> &masses,
			thrust::device_vector<float3> &velocities, void *&tempStorage, uint64_t &tempStorageSize);



		void ProjectConstraints(thrust::device_vector<float3> &positions, const thrust::device_vector<float> &invMasses,
			const int iteration);


		void ApplyCorrections(thrust::device_vector<float3> &positions);

		void FinalUpdate(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions,
			thrust::device_vector<float3> &velocities, const thrust::device_vector<bool> &fixedVerts);
	};


	
	__global__ void ProjectStaticConstraints(const int stretchCount, const int bendCount, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ sid1, const int * __restrict__ sid2, const int * __restrict__ sid1o, const int * __restrict__ sid2o,
		const float sk, const float * __restrict__ sl0,
		const int * __restrict__ bid1, const int * __restrict__ bid2, const int * __restrict__ bid3, const int * __restrict__ bid4,
		const int * __restrict__ bid1o, const int * __restrict__ bid2o, const int * __restrict__ bid3o, const int * __restrict__ bid4o,
		const float bk, const float * __restrict__ bphi0, float3 * __restrict__ corVals);
	

	__device__ void ProjectStaticStretchConstraint(const int id, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ sid1, const int * __restrict__ sid2,
		const int * __restrict__ sid1o, const int * __restrict__ sid2o,
		const float sk, const float * __restrict__ sl0, float3 * __restrict__ corVals);

	__device__ void ProjectStaticBendConstraint(const int id, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ bid1, const int * __restrict__ bid2, const int * __restrict__ bid3, const int * __restrict__ bid4,
		const int * __restrict__ bid1o, const int * __restrict__ bid2o, const int * __restrict__ bid3o, const int * __restrict__ bid4o,
		const float bk, const float * __restrict__ bphi0, float3 * __restrict__ corVals);		
	

	__global__ void _ApplyCorrections(const int particleCount, const float3 * __restrict__ rawValues, 
		const int * __restrict__ prefixSums,
		float3 * __restrict__ positions);

	__global__ void _FinalUpdate(const int particleCount, float3 * __restrict__ positions, float3 * __restrict__ prevPositions,
		float3 * __restrict__ velocities, const bool * __restrict__ fixedVerts, const float timestep);

	__global__ void ComputeXMVMs(const int particleCount, const float3 * __restrict__ pos, const float3 * __restrict__ vel,
		const float * __restrict__ mass,
		float3 * __restrict__ XMs, float3 * __restrict__ VMs);


	__global__ void ComputeL(const int particleCount, const float3 *__restrict__ pos, const float3 xcm, float3 * __restrict__ vimi, float3 *__restrict__ out);

	__global__ void ComputeIs(const int particleCount, const float3 *__restrict__ pos, const float3 xcm,
		const float * __restrict__ mass, float3 * __restrict__ aux1, float3 * __restrict__ aux2);

	__global__ void UpdateVels(const int particleCount, const float3 * __restrict__ pos,
		const float3 omega, const float3 xcm, const float3 vcm, const float dampingCoef, float3 * __restrict__ vel);

	__global__ void ApplyExternalForces(const int particleCount, float3 * __restrict__ vels, const float3 gravity, const float timestep);

	__global__ void ExplicitEuler(const int particleCount, const float3 * __restrict__ prevPositions, const float3 * __restrict__ velocities,
		float3 * __restrict__ positions, const float timestep);

}