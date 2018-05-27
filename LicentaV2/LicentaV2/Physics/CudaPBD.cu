#include "CudaPBD.cuh"

#include "../Dependencies/glm/glm.hpp"

#include "../Core/CubWrappers.cuh"

void CudaPBD::DampVelocities(const thrust::device_vector<float3>& positions, const thrust::device_vector<float>& masses,
	const float dampingCoef, thrust::device_vector<float3>& velocities, thrust::device_vector<float3> &aux1,
	thrust::device_vector<float3> &aux2,
	void *&tempStorage, uint64_t tempStorageSize)
{
	const int particleCount = positions.size();
	const int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;


	//float massSum = CubWrap::ReduceSum(masses, tempStorage, tempStorageSize);

	float vertexMass;

	cudaMemcpy(&vertexMass, cu::raw(masses), sizeof(float), cudaMemcpyDeviceToHost);
	
	const float massSum = vertexMass * masses.size();


	ComputeXMVMs <<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>> (particleCount, cu::raw(positions), cu::raw(velocities), cu::raw(masses), cu::raw(aux1), cu::raw(aux2));

	const float3 ximiSum = CubWrap::ReduceSum(aux1, tempStorage, tempStorageSize);
	const float3 vimiSum = CubWrap::ReduceSum(aux2, tempStorage, tempStorageSize);

	const float3 xcm = ximiSum / massSum;
	const float3 vcm = vimiSum / massSum;


	ComputeL << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), xcm, cu::raw(aux2), cu::raw(aux1));

	const float3 L = CubWrap::ReduceSum(aux1, tempStorage, tempStorageSize);

	ComputeIs << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), xcm, cu::raw(masses), cu::raw(aux1), cu::raw(aux2));

	const float3 i1 = CubWrap::ReduceSum(aux1, tempStorage, tempStorageSize);
	const float3 i2 = CubWrap::ReduceSum(aux2, tempStorage, tempStorageSize);

	const glm::mat3 I(glm::vec3(i1.y + i1.z, i2.x, i2.y),
		glm::vec3(i2.x, i1.x + i1.z, i2.z),
		glm::vec3(i2.y, i2.z, i1.x + i1.y));


	glm::vec3 omegaGLM = glm::inverse(I) * CudaUtils::MakeVec(L);

	const float3 omega = make_float3(omegaGLM.x, omegaGLM.y, omegaGLM.z);

	UpdateVels << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), omega, xcm, vcm, dampingCoef, cu::raw(velocities));
	
}

__global__ void CudaPBD::ComputeXMVMs(const int particleCount, const float3 *__restrict__ pos, const float3 * __restrict__ vel,
	const float *__restrict__ mass, float3 * __restrict__ XMs, float3 * __restrict__ VMs)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	const float m = mass[id];
	XMs[id] = pos[id] * m;
	VMs[id] = vel[id] * m;
}

__global__ void CudaPBD::ComputeL(const int particleCount, const float3 *__restrict__ pos, const float3 xcm, float3 * __restrict__ vimi, float3 *__restrict__ out)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	float3 ri = pos[id] - xcm;

	out[id] = CudaUtils::cross(ri, vimi[id]);
}

__global__ void CudaPBD::ComputeIs(const int particleCount, const float3 *__restrict__ pos, const float3 xcm, const float *__restrict__ mass, float3 *__restrict__ aux1, float3 *__restrict__ aux2)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	float3 ri = pos[id] - xcm;

	aux1[id] = make_float3(ri.x * ri.x, ri.y * ri.y, ri.z * ri.z) * mass[id];
	aux2[id] = -make_float3(ri.x * ri.y, ri.x * ri.z, ri.y * ri.z) * mass[id];
}

__global__ void CudaPBD::UpdateVels(const int particleCount, const float3 *__restrict__ pos, const float3 omega, const float3 xcm,
	const float3 vcm, const float dampingCoef, float3 *__restrict__ vel)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	float3 ri = pos[id] - xcm;

	float3 dv = vcm + CudaUtils::cross(omega, ri) - vel[id];

	vel[id] = vel[id] + dampingCoef * dv;
}

