#include "CudaMassSpring.cuh"

#include "../Core/CudaUtils.cuh"

void CudaMassSpring::ClothEngineStep(thrust::device_vector<float3>& positions, thrust::device_vector<float3>& prevPositions, thrust::device_vector<float3>& velocities, 
	const thrust::device_vector<float>& masses, const thrust::device_vector<int>& aIDs, const thrust::device_vector<int>& bIDs, 
	const thrust::device_vector<float>& ks, const thrust::device_vector<float>& l0s, const thrust::device_vector<int>& springInfo, 
	const thrust::device_vector<int>& springIDs, const float3 gravity, const float dampingCoef, const float timeStep, const float bendcoef,
	const thrust::device_vector<bool>& fixedVerts)
{
	const int particleCount = positions.size();
	const int springCount = aIDs.size();	

	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ClothEngineStep<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(particleCount, thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&prevPositions[0]),
		thrust::raw_pointer_cast(&velocities[0]), thrust::raw_pointer_cast(&masses[0]), springCount, thrust::raw_pointer_cast(&aIDs[0]), thrust::raw_pointer_cast(&bIDs[0]),
		thrust::raw_pointer_cast(&ks[0]), thrust::raw_pointer_cast(&l0s[0]), thrust::raw_pointer_cast(&springInfo[0]), thrust::raw_pointer_cast(&springIDs[0]),
		gravity, dampingCoef, timeStep, bendcoef, thrust::raw_pointer_cast(&fixedVerts[0]));
}

__global__ void CudaMassSpring::ClothEngineStep(const int particleCount, float3 * __restrict__ positions, float3 *__restrict__ prevPositions, float3 *__restrict__ velocities,
	const float *__restrict__ masses, const int springCount, const int *__restrict__ aIDs, const int *__restrict__ bIDs, const float *__restrict__ ks, const float *__restrict__ l0s,
	const int *__restrict__ springInfo, const int *__restrict__ springIDs, const float3 gravity, const float dampingCoef, const float timeStep, const float bendCoef,
	const bool *__restrict__ fixedVerts)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	if (fixedVerts[id])
		return;


	//float3 force = masses[id] * gravity - dampingCoef * velocities[id];
	float3 force = masses[id] * gravity - dampingCoef * (positions[id] - prevPositions[id]);

	for (int i = springIDs[id]; i < springIDs[id + 1]; ++i)
	{

		int sprID = springInfo[i];
		float3 dir = positions[aIDs[sprID]] - positions[bIDs[sprID]];

		if (CudaUtils::len(dir) == 0)
		{

			printf("[%d] dir is 0, aid: %d, bid: %d\n", id, aIDs[sprID], bIDs[sprID]);
			//dir = make_float3(1.f, 1.f, 1.f);
		}
		//float len = norm3df(dir.x, dir.y, dir.z);

		//float k = ComputeSpringDeformation(positions[aIDs[sprID]], positions[bIDs[sprID]], l0s[sprID]) > 0.1f && ks[sprID] != bendCoef ? 0.99f : ks[sprID];

		if (CudaUtils::len(dir) > l0s[sprID])
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

	if (CudaUtils::isNan(newPos))
	{
		printf("NEWPOS NAN\n");
	}

	prevPositions[id] = positions[id];
	positions[id] = newPos;

}

__device__ float CudaMassSpring::ComputeSpringDeformation(const float3 a, const float3 b, const float l0)
{
	return (CudaUtils::distance(a, b) - l0) / l0;
}

void CudaMassSpring::AdjustSprings(thrust::device_vector<float3>& positions, const thrust::device_vector<int>& bIDs, const thrust::device_vector<float>& l0s, const float maxDeformation,
	const thrust::device_vector<int>& springInfo, const thrust::device_vector<int>& springIDs, const thrust::device_vector<bool>& fixedVerts, 
	const thrust::device_vector<float>& ks, const float bendCoef)
{
	const int particleCount = positions.size();	

	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	for (int i = 0; i < 5; ++i)
	{
		_AdjustSprings << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&bIDs[0]),
			thrust::raw_pointer_cast(&l0s[0]), maxDeformation, thrust::raw_pointer_cast(&springInfo[0]), thrust::raw_pointer_cast(&springIDs[0]), thrust::raw_pointer_cast(&fixedVerts[0]),
			thrust::raw_pointer_cast(&ks[0]), bendCoef);
	}
}

__global__ void CudaMassSpring::_AdjustSprings(const int particleCount, float3 * __restrict__ positions,
	const int * __restrict__ bIDs, const float * __restrict__ l0s, const float maxDeformation, const int * __restrict__ springInfo, const int * __restrict__ springIDs,
	const bool * __restrict__ fixedVerts, const float * __restrict__ ks, const float bendCoef)
{
	int id = CudaUtils::MyID();

	if (id >= particleCount)
		return;

	if (fixedVerts[id])
		return;

	//float3 correction = make_float3(0.f, 0.f, 0.f);

	for (int i = springIDs[id]; i < springIDs[id + 1]; ++i)
	{
		int sprID = springInfo[i];

		float deformation = ComputeSpringDeformation(positions[id], positions[bIDs[sprID]], l0s[sprID]);


		if (deformation > maxDeformation && ks[sprID] != bendCoef)
		{

			float dif = (deformation - maxDeformation) * l0s[sprID];

			//float3 dir = positions[bIDs[sprID]] - positions[id];

			//float len = CudaUtils::distance(positions[id], positions[bIDs[sprID]]);

			//float dif = (len - l0s[sprID]) / len;

			//printf("[%d] Deformation = %f, bid: %d, max: %d, dif: %f\n", id, deformation, bIDs[sprID], particleCount, dif);

			float3 push = CudaUtils::normalize(positions[bIDs[sprID]] - positions[id]) * dif / (fixedVerts[bIDs[sprID]] ? 1.f : 2.f);

			//float3 push = dir * (fixedVerts[bIDs[sprID]] ? 1.f : 0.5f) * dif;

			//printf("[%d] Push = (%f, %f, %f)\n", id, push.x, push.y, push.z);
			//printf("[%d] dif = %f\n", id, dif);


			//correction = correction + push;
			positions[id] = positions[id] + push;
		}
	}

	//positions[id] = positions[id] + correction;
}



//
//
//
//
//
//
//
//__global__ void AdjustSprings(const int particleCount, float3 *positions, const int springCount, int *aIDs, int *bIDs, float *l0s, const float maxDeformation, bool *fixedVerts)
//{
//	int id = CudaUtils::MyID();
//
//	if (id >= springCount)
//		return;
//
//	int aID = aIDs[id], bID = bIDs[id];
//	float l0 = l0s[id];
//
//	float deformation = ComputeSpringDeformation(positions[aID], positions[bID], l0);
//
//	if (deformation > maxDeformation)
//	{
//		bool aFixed = fixedVerts[aID], bFixed = fixedVerts[bID];
//
//		if (aFixed && bFixed)
//			return;
//
//		float dif = (deformation - maxDeformation) * l0;
//		float3 push = CudaUtils::normalize(positions[bID] - positions[aID]) * dif;
//
//		if (!aFixed && bFixed)
//		{
//			atomicAdd(&(positions[aIDs[id]].x), push.x / 2);
//			atomicAdd(&(positions[aIDs[id]].y), push.y / 2);
//			atomicAdd(&(positions[aIDs[id]].z), push.z / 2);
//
//			//positions[aIDs[id]] = positions[aIDs[id]] + push / 2;
//		}
//		else if (aFixed && !bFixed)
//		{
//			//positions[bIDs[id]] = positions[bIDs[id]] - push / 2;
//
//			atomicAdd(&(positions[bIDs[id]].x), -push.x / 2);
//			atomicAdd(&(positions[bIDs[id]].y), -push.y / 2);
//			atomicAdd(&(positions[bIDs[id]].z), -push.z / 2);
//		}
//		else
//		{
//			atomicAdd(&(positions[aIDs[id]].x), push.x / 4);
//			atomicAdd(&(positions[aIDs[id]].y), push.y / 4);
//			atomicAdd(&(positions[aIDs[id]].z), push.z / 4);
//
//			atomicAdd(&(positions[bIDs[id]].x), -push.x / 4);
//			atomicAdd(&(positions[bIDs[id]].y), -push.y / 4);
//			atomicAdd(&(positions[bIDs[id]].z), -push.z / 4);
//
//			//positions[aIDs[id]] = positions[aIDs[id]] + push / 4;
//			//positions[bIDs[id]] = positions[bIDs[id]] - push / 4;
//		}
//	}
//
//}
//