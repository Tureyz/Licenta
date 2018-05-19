#pragma once

#include <thrust/device_vector.h>

namespace Physics
{
	struct AABBCollision
	{
		int m_id1;
		int m_id2;
		uint64_t m_timestamp;

		__host__ __device__ AABBCollision(const int id1, const int id2, const uint64_t ts) : m_id1(id1), m_id2(id2), m_timestamp(ts) {}
		__host__ __device__ AABBCollision() : m_id1(-1), m_id2(-2), m_timestamp(0) {}
	};

	struct PrimitiveContact
	{
		int v1, v2, v3, v4;
		float w1, w2, w3, w4;
		float t;
		float3 n;
		bool contact;

		__host__ __device__ PrimitiveContact() : v1(-1), v2(-1), v3(-1), v4(-1), w1(-1), w2(-1), w3(-1), w4(-1),
			t(-1), contact(false) {}
		__host__ __device__ PrimitiveContact(const int v1, const int v2, const int v3, const int v4) :
			v1(v1), v2(v2), v3(v3), v4(v4) {}
	};

	struct CudaSpring
	{
		int m_aID;
		int m_bID;
		float m_stiffness;
		float m_initialLength;
	};

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

	struct CudaTriangle
	{
		/// IDs
		int m_v1;
		int m_v2;
		int m_v3;
		int m_aabbMin;
		int m_aabbMax;

		/// Actual normal
		float3 m_faceNormal;
	};

}