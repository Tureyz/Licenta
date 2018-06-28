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

	struct TriangleBendConstraintList
	{
		// id1 = b0
		thrust::device_vector<int> id1;

		// id2 = b1
		thrust::device_vector<int> id2;

		// id3 = v
		thrust::device_vector<int> id3;

		thrust::device_vector<int> id1Offset;
		thrust::device_vector<int> id2Offset;
		thrust::device_vector<int> id3Offset;

		thrust::device_vector<float> h0;
		float k;
	};

	struct GroundConstraintList
	{
		
		thrust::device_vector<bool> flag;
		thrust::device_vector<float3> qc;
		thrust::device_vector<float3> nc;
	};

	struct LRAConstraintList
	{
		int fixedVertCount;
		thrust::device_vector<bool> flag;
		thrust::device_vector<int> fixedPosID;
		thrust::device_vector<float> l0;
	};

	struct CudaPBD
	{
		Physics::ClothParams m_parentParams;

		StretchConstraintList m_stretchConstraints;
		BendConstraintList m_bendConstraints;
		TriangleBendConstraintList m_triangleBendConstraints;

		GroundConstraintList m_groundConstraints;
		GroundConstraintList m_sphereConstraints;
		LRAConstraintList m_LRAConstraints;


		//thrust::device_vector<int> m_staticCorrectionIDs;
		thrust::device_vector<float3> m_staticCorrectionValues;
		thrust::device_vector<int> m_staticCorrectionPrefixSums;
		thrust::device_vector<float3> m_accumulatedCorrectionValues;
		thrust::device_vector<int> m_accumulatedCorrectionCounts;

		thrust::device_vector<float3> m_aux1;
		thrust::device_vector<float3> m_aux2;

		Physics::DoubleBuffer<uint32_t> m_dbDynamicCorrectionID;
		Physics::DoubleBuffer<float3> m_dbDynamicCorrectionValues;
		

		uint64_t m_dynamicCorrectionsSize;
		int m_dynamicCorrectionsRunCount;

		thrust::device_vector<uint32_t> m_dDynamicCorrectionRLEUniques;
		thrust::device_vector<int> m_dDynamicCorrectionRLECounts;


		Physics::DoubleBuffer<uint32_t> m_dbtocID;
		Physics::DoubleBuffer<float> m_dbtoc;

		uint64_t m_tocSize;
		int m_tocRunCount;

		thrust::device_vector<uint32_t> m_dtocRLEUniques;
		thrust::device_vector<int> m_dtocRLECounts;


		Physics::DoubleBuffer<uint32_t> m_dbFrictionCorrectionID;
		Physics::DoubleBuffer<float3> m_dbFrictionCorrectionValues;

		uint64_t m_frictionCorrectionsSize;
		int m_frictionCorrectionsRunCount;

		thrust::device_vector<uint32_t> m_dFrictionCorrectionRLEUniques;
		thrust::device_vector<int> m_dFrictionCorrectionRLECounts;

		thrust::device_vector<float3> m_accumulatedVelCorrectionValues;
		thrust::device_vector<int> m_accumulatedVelCorrectionCounts;

		int m_particleCount;

		float3 m_spherePos;
		float m_sphereRadius;
		bool m_spherePresent;

		float3 m_crtWindDir;

		void SetSpherePos(const float3 &position, const float radius);

		void CreateTriangleBendingConstraints(const std::vector<Rendering::VertexFormat> &verts,
			thrust::host_vector<int> & id1, thrust::host_vector<int> & id2, thrust::host_vector<int> & id3,
			thrust::host_vector<float> & h0);

		int lin(const int i, const int j);

		int lin(const int2 coord);

		int2 delin(const int i);

		std::vector<int> GetNeighbors(const int2 coord);

		bool InBounds(const int i, const int j);

		bool InBounds(const int2 coord);

		void PBDStepExternal(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions,
			const thrust::device_vector<float> &masses,
			thrust::device_vector<float3> &velocities, const thrust::device_vector<float3> &vertexNormals,
			void *&tempStorage, uint64_t &tempStorageSize);


		void PBDStepSolver(thrust::device_vector<float3> &positions, thrust::device_vector<float3>& prevPositions,
			const thrust::device_vector<float> &invMasses,
			const thrust::device_vector<Physics::PrimitiveContact>& vfContacts, const uint64_t & vfContactsSize,
			const thrust::device_vector<Physics::PrimitiveContact>& eeContacts, const uint64_t & eeContactsSize,
			void *&tempStorage, uint64_t &tempStorageSize);

		void PBDStepIntegration(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions,
			thrust::device_vector<float3> &velocities,
			const thrust::device_vector<bool> &fixedVerts);

		void Init(const Physics::ClothParams parentParams, const std::vector<Rendering::VertexFormat> &verts, const std::vector<unsigned int> &indices);

		void CreateStaticStretchConstraint(const int id1, const int id2, const std::vector<Rendering::VertexFormat> &verts,
			thrust::host_vector<int> &sid1, thrust::host_vector<int> &sid2, thrust::host_vector<float> &sl0);

		void CreateStaticBendConstraint(const int3 tri1, const int3 tri2, const std::vector<Rendering::VertexFormat> &verts,
			thrust::host_vector<int> &bid1, thrust::host_vector<int> &bid2, thrust::host_vector<int> &bid3,
			thrust::host_vector<int> &bid4,
			thrust::host_vector<float> &bphi0);

		void CreateGroundConstraints(const thrust::device_vector<float3> &positions,
			const thrust::device_vector<float3> &prevPositions,
			const thrust::device_vector<float> &invMasses);

		void CreateSphereConstraints(thrust::device_vector<float3> &positions,
			thrust::device_vector<float3> &prevPositions,
			const thrust::device_vector<float> &invMasses);

		void CreateLRAConstraints(const thrust::device_vector<float3> &positions);

		void DampVelocities(const thrust::device_vector<float3> &positions, const thrust::device_vector<float> &masses,
			thrust::device_vector<float3> &velocities, void *&tempStorage, uint64_t &tempStorageSize);


		void ProjectDynamicConstraints(const thrust::device_vector<float3> &positions,
			const thrust::device_vector<float3> &prevPositions,
			const thrust::device_vector<float> &invMasses, 
			const float thickness,
			const thrust::device_vector<Physics::PrimitiveContact>& vfContacts,
			const uint64_t vfContactsSize,
			const thrust::device_vector<Physics::PrimitiveContact>& eeContacts,
			const uint64_t eeContactsSize,
			void *& tempStorage, uint64_t & tempStorageSize);

		void RevertToTOC(thrust::device_vector<float3> &positions, const thrust::device_vector<float3>& prevPositions,
			const thrust::device_vector<Physics::PrimitiveContact>& vfContacts, const uint64_t & vfContactsSize,
			const thrust::device_vector<Physics::PrimitiveContact>& eeContacts, const uint64_t & eeContactsSize,
			void *&tempStorage, uint64_t &tempStorageSize);


		void ApplyFriction(thrust::device_vector<float3> &velocities,
			const thrust::device_vector<Physics::PrimitiveContact>& vfContacts,
			const uint64_t vfContactsSize,
			const thrust::device_vector<Physics::PrimitiveContact>& eeContacts,
			const uint64_t eeContactsSize,
			void *& tempStorage, uint64_t & tempStorageSize);


		void ProjectStaticConstraints(const thrust::device_vector<float3> &positions, const thrust::device_vector<float> &invMasses,
			const int iteration);

		void ProjectGroundConstraints(const thrust::device_vector<float3> &positions,
			const thrust::device_vector<float> &invMasses);

		void ProjectSphereConstraints(thrust::device_vector<float3> &positions,
			const thrust::device_vector<float> &invMasses);

		void ProjectLRAConstraints(const thrust::device_vector<float3> &positions,
			const thrust::device_vector<float> &invMasses);

		void ApplyCorrections(thrust::device_vector<float3> &positions);

		void FinalUpdate(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions,
			thrust::device_vector<float3> &velocities, const thrust::device_vector<bool> &fixedVerts);



	};


	__global__ void _CreateGroundConstraints(const int particleCount,
		const float3 * __restrict__ positions, const float3 * __restrict__ prevPositions,
		const float * __restrict__ invMasses, const float groundHeight, const float thickness,
		float3 * __restrict__ qcs, float3 * __restrict__ ncs, bool * __restrict__ flags);

	__global__ void _CreateSphereConstraints(const int particleCount,
		float3 * __restrict__ positions, float3 * __restrict__ prevPositions,
		const float * __restrict__ invMasses, const float3 spherePos, const float sphereRadius, const float thickness,
		float3 * __restrict__ qcs, float3 * __restrict__ ncs, bool * __restrict__ flags);
		
	__global__ void _CreateLRAConstraints(const int particleCount, const int fixedCount, 
		const float3 * __restrict__ positions, const bool * __restrict__ fixedVerts, const int * __restrict__ fixedPosIDs,
		const float * __restrict__ l0s, bool * __restrict__ flags);
	

	__global__ void _ProjectStaticConstraints(const int stretchCount, const int bendCount, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ sid1, const int * __restrict__ sid2, const int * __restrict__ sid1o, const int * __restrict__ sid2o,
		const float sk, const float * __restrict__ sl0,
		const int * __restrict__ bid1, const int * __restrict__ bid2, const int * __restrict__ bid3, const int * __restrict__ bid4,
		const int * __restrict__ bid1o, const int * __restrict__ bid2o, const int * __restrict__ bid3o, const int * __restrict__ bid4o,
		const float bk, const float * __restrict__ bphi0, float3 * __restrict__ corVals);	


	__global__ void _ProjectStaticConstraintsTriBend(const int stretchCount, const int bendCount, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ sid1, const int * __restrict__ sid2, const int * __restrict__ sid1o, const int * __restrict__ sid2o,
		const float sk, const float * __restrict__ sl0,
		const int * __restrict__ bid1, const int * __restrict__ bid2, const int * __restrict__ bid3,
		const int * __restrict__ bid1o, const int * __restrict__ bid2o, const int * __restrict__ bid3o,
		const float bk, const float * __restrict__ bh0, float3 * __restrict__ corVals);

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

	__device__ void ProjectStaticTriangleBendConstraint(const int id, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ bid1, const int * __restrict__ bid2, const int * __restrict__ bid3,
		const int * __restrict__ bid1o, const int * __restrict__ bid2o, const int * __restrict__ bid3o,
		const float bk, const float * __restrict__ bh0, float3 * __restrict__ corVals);

	
	__global__ void _ProjectDynamicConstraints(const float3 * __restrict__ positions,
		const float3 * __restrict__ prevPositions,
		const float * __restrict__ invMasses,
		const float thickness,
		const Physics::PrimitiveContact * __restrict__ vfs, const uint64_t vfSize,
		const Physics::PrimitiveContact * __restrict__ ees, const uint64_t eeSize,
		uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals);




	__device__ void ProjectVFConstraint(const int id, const int myStart,
		const float3 * __restrict__ positions,
		const float3 * __restrict__ prevPositions,
		const float * __restrict__ invMasses, const float thickness,
		const Physics::PrimitiveContact * __restrict__ vfs,
		uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals);

	__device__ void ProjectEEConstraint(const int id, const int myStart,
		const float3 * __restrict__ positions,
		const float3 * __restrict__ prevPositions,
		const float * __restrict__ invMasses, const float thickness,
		const Physics::PrimitiveContact * __restrict__ ees,
		uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals);

	__global__ void _ApplyStaticCorrections(const int particleCount, const float3 * __restrict__ rawValues, 
		const int * __restrict__ prefixSums,
		float3 * __restrict__ accumulatedValues, int * __restrict__ accumulatedCounts);


	__global__ void _ApplyDynamicCorrections(const int runCount, const uint32_t * __restrict__ RLEUniques,
		const int * __restrict__ RLEPrefixSums,
		const float3 * __restrict__ rawCorVals, const uint64_t rawCorValsSize, float3 * __restrict__ accumulatedCors,
		int * __restrict__ accumulatedCounts);


	__global__ void _ApplyGroundCorrections(const int particleCount, const float3 * __restrict__ positions,
		const float * __restrict__ invMasses, const bool * __restrict__ flags, const float3 * __restrict__ qcs,
		float3 * __restrict__ ncs,
		float3 * __restrict__ accumulatedCors, int * __restrict__ accumulatedCounts);

	__global__ void _ApplySphereCorrections(const int particleCount, float3 * __restrict__ positions,
		const float thickness, const bool * __restrict__ flags, const float3 * __restrict__ qcs,
		float3 * __restrict__ ncs,
		float3 * __restrict__ accumulatedCors, int * __restrict__ accumulatedCounts);

	__global__ void _ApplyLRACorrections(const int particleCount, const int fixedCount,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ fixedIDs, const float * __restrict__ l0s,
		const bool * __restrict__ flags,
		float3 * __restrict__ accumulatedCors, int * __restrict__ accumulatedCounts);

	__global__ void FinalCorrectionStep(const int particleCount,
		const float3 * __restrict__ accumulatedCors, const int * __restrict__ accumulatedCounts,
		float3 * __restrict__ positions);

	__global__ void FinalFrictionStep(const int particleCount, const float kFriction,
		const float3 * __restrict__ accumulatedCors, const int * __restrict__ accumulatedCounts,
		float3 * __restrict__ velocities);


	__global__ void _ApplyFriction(const float3 * __restrict__ vels, const float kFriction,
		const Physics::PrimitiveContact * __restrict__ vfs, const uint64_t vfSize,
		const Physics::PrimitiveContact * __restrict__ ees, const uint64_t eeSize,
		uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals);

	__device__ void _ApplyFrictionVF(const int id, const int myStart, const float kFriction,
		const float3 * __restrict__ velocities, const Physics::PrimitiveContact * __restrict__ vfs,
		uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals);

	__device__ void _ApplyFrictionEE(const int id, const int myStart, const float kFriction,
		const float3 * __restrict__ velocities, const Physics::PrimitiveContact * __restrict__ ees,
		uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals);


	__global__ void _AccumulateFriction(const int runCount, const float kFriction,
		const uint32_t * __restrict__ RLEUniques,
		const int * __restrict__ RLEPrefixSums,
		const float3 * __restrict__ rawCorVals, const uint64_t rawCorValsSize, float3 * __restrict__ accumulatedCors,
		int * __restrict__ accumulatedCounts);


	__global__ void _DampVelRigid(const int particleCount, float3 * __restrict__ vels, const float coef,
		const bool * __restrict__ groundFlags, const float3 * __restrict__ groundNormals,
		const bool * __restrict__ sphereFlags, const float3 * __restrict__ sphereNormals);




	__global__ void _FinalUpdate(const int particleCount, float3 * __restrict__ positions, float3 * __restrict__ prevPositions,
		float3 * __restrict__ velocities, const bool * __restrict__ fixedVerts, const float timestep);






	__global__ void GlobalVelDamp(const int particleCount, float3 * __restrict__ vel, const float kGlobalDamp);


	__global__ void ComputeXMVMs(const int particleCount, const float3 * __restrict__ pos, const float3 * __restrict__ vel,
		const float * __restrict__ mass,
		float3 * __restrict__ XMs, float3 * __restrict__ VMs);


	__global__ void ComputeL(const int particleCount, const float3 *__restrict__ pos, const float3 xcm, float3 * __restrict__ vimi, float3 *__restrict__ out);

	__global__ void ComputeIs(const int particleCount, const float3 *__restrict__ pos, const float3 xcm,
		const float * __restrict__ mass, float3 * __restrict__ aux1, float3 * __restrict__ aux2);

	__global__ void UpdateVels(const int particleCount, const float3 * __restrict__ pos,
		const float3 omega, const float3 xcm, const float3 vcm, const float dampingCoef, float3 * __restrict__ vel);

	__global__ void ApplyExternalForces(const int particleCount, float3 * __restrict__ vels, const float3 * __restrict__ particleNormals,
		const float3 gravity, const float3 wind, const float timestep);

	__global__ void ExplicitEuler(const int particleCount, const float3 * __restrict__ prevPositions, const float3 * __restrict__ velocities,
		float3 * __restrict__ positions, const float timestep);

	__global__ void FillTOCs(const Physics::PrimitiveContact * __restrict__ vfs, const uint64_t vfSize,
		const Physics::PrimitiveContact * __restrict__ ees, const uint64_t eeSize,
		uint32_t * __restrict__ rawtocIDs, float * __restrict__ rawtocVals);

	__device__ void FillTOC(const int id, const int myStart, const Physics::PrimitiveContact * __restrict__ contacts,
		uint32_t * __restrict__ rawtocIDs, float * __restrict__ rawtocVals);

	__global__ void _RevertToTOC(const int runCount, const float3 * __restrict__ prevPositions, 
		const uint32_t * __restrict__ RLEUniques,
		const int * __restrict__ RLEPrefixSums,
		const float * __restrict__ rawtocVals, const uint64_t rawtocValsSize, float3 * __restrict__ positions);

}










//void CreateStaticBendConstraint2(const int3 tri1, const int3 tri2, const std::vector<Rendering::VertexFormat> &verts,
//	thrust::host_vector<int> &bid1, thrust::host_vector<int> &bid2, thrust::host_vector<int> &bid3,
//	thrust::host_vector<int> &bid4,
//	thrust::host_vector<float> &bphi0);

//void ProjectConstraints(thrust::device_vector<float3> &positions, const thrust::device_vector<float> &invMasses,
//	const int iteration,
//	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts,
//	const uint64_t & vfContactsSize,
//	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts,
//	const uint64_t & eeContactsSize,
//	void *& tempStorage, uint64_t & tempStorageSize);


	/*__device__ void ProjectStaticBendConstraint2(const int id, const int iteration,
		const float3 * __restrict__ positions, const float * __restrict__ invMasses,
		const int * __restrict__ bid1, const int * __restrict__ bid2, const int * __restrict__ bid3, const int * __restrict__ bid4,
		const int * __restrict__ bid1o, const int * __restrict__ bid2o, const int * __restrict__ bid3o, const int * __restrict__ bid4o,
		const float bk, const float * __restrict__ bphi0, float3 * __restrict__ corVals);*/