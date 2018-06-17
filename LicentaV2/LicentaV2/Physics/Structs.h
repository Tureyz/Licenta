#pragma once

#include <thrust/device_vector.h>

namespace Physics
{
	/*struct ImpulseInfo
	{
		int particleID;
		float3 impulse;

		__host__ __device__ ImpulseInfo(const int id, const float3 impulse) : particleID(id), impulse(impulse) {}

		__host__ __device__ ImpulseInfo() : particleID(-1)
		{
			impulse.x = 0.f;
			impulse.y = 0.f;
			impulse.z = 0.f;
		}
	};*/


	struct ClothParams
	{
		float timestep;
		float kFriction;
		float kStretch;
		float kShear;
		float kBend;
		float kDamp;
		float kSpringDamp;
		float thickness;
		float objectMass;
		float globalVelDamp;
		float strainLimit;

		float3 gravity;

		float3 minWindDir;
		float3 startWindDir;
		float3 maxWindDir;
		float windMinVariation;
		float windMaxVariation;
		bool windOn;

		float3 worldMin;
		float3 worldMax;

		int2 dims;
		int solverIterations;
		int ccdIterations;
		int BVHChunkSize;

		thrust::device_vector<bool> fixedVerts;

		uint64_t benchmarkSample;

		bool useTriangleBending;
		bool colorContacts;
	};

	template <typename T>
	struct DoubleBuffer
	{
		thrust::device_vector<T> buffers[2];
		int selector;


		__host__ __device__ DoubleBuffer()
		{
			selector = 0;
		}

		__host__ __device__ DoubleBuffer(thrust::device_vector<T> &crt, thrust::device_vector<T> &alt)
		{
			selector = 0;
			buffers[1] = crt;
			buffers[2] = alt;
		}

		__host__ __device__ thrust::device_vector<T> & Current()
		{
			return buffers[selector];
		}

		__host__ __device__ void SwapBuffers()
		{
			selector = (selector + 1) % 2;
		}

		
		
	};
	struct AABBCollision
	{
		int m_id1;
		int m_id2;
		//uint64_t m_timestamp;

		__host__ __device__ AABBCollision(const int id1, const int id2/*, const uint64_t ts*/) :
			m_id1(id1), m_id2(id2)/*, m_timestamp(ts)*/ {}
		__host__ __device__ AABBCollision() : m_id1(-1), m_id2(-2)/*, m_timestamp(0) */{}
	};

	struct PrimitiveContact
	{
		int v1, v2, v3, v4;
		float w1, w2, w3, w4;
		float t;
		float3 n;
		//bool contact;

		__host__ __device__ PrimitiveContact() : v1(-1), v2(-1), v3(-1), v4(-1), w1(-1.f), w2(-1.f), w3(-1.f), w4(-1.f),
			t(-1.f)//, contact(false)
		{
			n.x = -1.f;
			n.y = -1.f;
			n.z = -1.f;
		}

		__host__ __device__ PrimitiveContact(const int v1, const int v2, const int v3, const int v4) :
			v1(v1), v2(v2), v3(v3), v4(v4), w1(-1.f), w2(-1.f), w3(-1.f), w4(-1.f), t(-1.f)//, contact(false)
		{
			n.x = -1.f;
			n.y = -1.f;
			n.z = -1.f;
		}
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

		/// 6 bits. 3 vertices, 3 edges. Edge order is (1 -> 2), (2 -> 3), (3 -> 1)
		char m_assignedFeatures;

		/// Actual normal
		float3 m_faceNormal;
	};

}