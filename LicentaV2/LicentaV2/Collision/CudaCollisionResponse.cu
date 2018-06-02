#include "CudaCollisionResponse.cuh"

#include "../Core/CubWrappers.cuh"

#include "CudaPrimitiveTests.cuh"



//#include <thrust/host_vector.h>

void DeformableUtils::CreateImpulses(const thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& velocities,
	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts, const uint64_t vfContactsSize,
	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts, const uint64_t eeContactsSize,
	Physics::DoubleBuffer<uint32_t>& impulseIDs, Physics::DoubleBuffer<float3>& impulseValues, thrust::device_vector<bool> &impulseFlags, uint64_t & impulsesSize,
	thrust::device_vector<uint32_t>& impulseRLEUniques, thrust::device_vector<int>& impulseRLECounts, int & impulseRunCount, thrust::device_vector<float3>& accumulatedImpulses,
	const float stiffness, const float vertexMass, const float timeStep, const float thickness, void *& tempStorage, uint64_t & tempStorageSize)
{
	cudaMemset(thrust::raw_pointer_cast(&accumulatedImpulses[0]), 0, accumulatedImpulses.size() * sizeof(float3));

	uint64_t totalImpulses = 4 * (vfContactsSize + eeContactsSize);

	if (totalImpulses == 0)
		return;

	if (impulsesSize < totalImpulses)
	{
		impulseIDs.buffers[0].resize(totalImpulses);
		impulseIDs.buffers[1].resize(totalImpulses);
		impulseValues.buffers[0].resize(totalImpulses);
		impulseValues.buffers[1].resize(totalImpulses);
		impulseRLEUniques.resize(totalImpulses);
		impulseRLECounts.resize(totalImpulses);
		impulseFlags.resize(totalImpulses);
		impulsesSize = totalImpulses;
	}




	int numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_CreateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (cu::raw(positions), cu::raw(velocities),
		cu::raw(vfContacts), vfContactsSize,
		cu::raw(eeContacts), eeContactsSize,
		cu::raw(impulseIDs.buffers[impulseIDs.selector]),
		cu::raw(impulseValues.buffers[impulseValues.selector]), cu::raw(impulseFlags), stiffness, vertexMass, timeStep, thickness);


	//uint64_t zeroCnt = 0;

	//thrust::host_vector<float3> hImpulses = impulseValues.buffers[impulseValues.selector];

	//for (int i = 0; i < totalImpulses; ++i)
	//{
	//	if (CudaUtils::isZero(hImpulses[i]))
	//		zeroCnt++;
	//}


	//printf("Before select %llu / %llu zeroes: (%.2f%%)\n", zeroCnt, totalImpulses, 100.0 *(((double)zeroCnt) / ((double)totalImpulses)));


	//int outImp = 0;

	//CubWrap::SelectFlagged(impulseIDs.buffers[impulseIDs.selector], impulseFlags, totalImpulses, impulseIDs.buffers[impulseIDs.selector], outImp, tempStorage, tempStorageSize);
	//CubWrap::SelectFlagged(impulseValues.buffers[impulseValues.selector], impulseFlags, totalImpulses, impulseValues.buffers[impulseValues.selector], outImp, tempStorage, tempStorageSize);

	//totalImpulses = (uint64_t) outImp;


	/*zeroCnt = 0;

	hImpulses = impulseValues.buffers[impulseValues.selector];

	for (int i = 0; i < totalImpulses; ++i)
	{
		if (CudaUtils::isZero(hImpulses[i]))
			zeroCnt++;
	}


	printf("After select %llu / %llu zeroes: (%.2f%%)\n", zeroCnt, totalImpulses, 100.0 *(((double)zeroCnt) / ((double)totalImpulses)));*/

	CubWrap::SortByKey(impulseIDs, impulseValues, totalImpulses, tempStorage, tempStorageSize);


	//thrust::host_vector<uint32_t> hids(impulseIDs.buffers[impulseIDs.selector].begin(), impulseIDs.buffers[impulseIDs.selector].begin() + totalImpulses);
	//thrust::host_vector<float3> hvals(impulseValues.buffers[impulseValues.selector].begin(), impulseValues.buffers[impulseValues.selector].begin() + totalImpulses);

	//for (int i = 0; i < totalImpulses - 1; ++i)
	//{
	//	if (hids[i] > hids[i + 1])
	//		printf("[%d] impulse: %d > %d -> (%f, %f, %f)\n", i, hids[i], hids[i + 1], hvals[i].x, hvals[i].y, hvals[i].z);
	//}


	CubWrap::RLE(impulseIDs.buffers[impulseIDs.selector], totalImpulses, impulseRLEUniques, impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);

	//thrust::host_vector<uint32_t> huniq(impulseRLEUniques.begin(), impulseRLEUniques.begin() + impulseRunCount);
	//thrust::host_vector<int> hcount(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);

	//
	//for (int i = 0; i < impulseRunCount; ++i)
	//{
	//	printf("RLE: %d -> %d\n", huniq[i], hcount[i]);
	//}

	CubWrap::ExclusiveSumInPlace(impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);

	//thrust::host_vector<int> hPref(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);
	//for (int i = 0; i < impulseRunCount; ++i)
	//{
	//	printf("Prefix: %d - %d\n", hcount[i], hPref[i]);
	//}

	//numBlocks = (impulseRunCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	//if (impulseRunCount > accumulatedImpulses.size())
	//{
	//	printf("EROAREOARE\n");

	//	for (int i = 0; i < impulseRunCount; ++i)
	//	{
	//		printf("[%d], RLE: %d -> %d\n", i, huniq[i], hcount[i]);
	//	}

	//	//for (int i = 0; i < totalImpulses - 1; ++i)
	//	//{
	//	//	if (hids[i] > hids[i + 1])
	//	//		printf("impulse: %d -> (%f, %f, %f)\n", hids[i], hvals[i].x, hvals[i].y, hvals[i].z);
	//	//}

	//	int z = 15;
	//}
	AccumulateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (impulseRunCount,
		thrust::raw_pointer_cast(&impulseRLEUniques[0]),
		thrust::raw_pointer_cast(&impulseRLECounts[0]),
		thrust::raw_pointer_cast(&impulseValues.buffers[impulseValues.selector][0]),
		totalImpulses, thrust::raw_pointer_cast(&accumulatedImpulses[0]));
}

__global__ void DeformableUtils::_CreateImpulses(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
	const Physics::PrimitiveContact *__restrict__ vfs, const uint64_t vfSize,
	const Physics::PrimitiveContact *__restrict__ ees, const uint64_t eeSize,
	uint32_t *__restrict__ impulseIDs, float3 *__restrict__ impulseValues, bool * __restrict__ impulseFlags, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{
	int id = CudaUtils::MyID();
	if (id >= vfSize + eeSize)
		return;

	if (id < vfSize)
	{
		_CreateImpulseVF(positions, velocities, vfs[id], impulseIDs, impulseValues, impulseFlags, id * 4, stiffness, vertexMass, timeStep, thickness);
	}
	else
	{
		_CreateImpulseEE(positions, velocities, ees[id - vfSize], impulseIDs, impulseValues, impulseFlags, id * 4, stiffness, vertexMass, timeStep, thickness);
	}
}

__device__ void DeformableUtils::_CreateImpulseVF(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
	const Physics::PrimitiveContact & contact,
	uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, bool * __restrict__ impulseFlags, const int myImpulseStart, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{

	float3 cv1 = contact.w1 * velocities[contact.v1];
	float3 cv2 = contact.w2 * velocities[contact.v2] + contact.w3 * velocities[contact.v3] + contact.w4 * velocities[contact.v4];
	float3 vr = cv1 + cv2;

	_CreateImpulse(positions, contact, vr, impulseIDs, impulseValues, impulseFlags, myImpulseStart, stiffness, vertexMass, timeStep, thickness);

}

__device__ void DeformableUtils::_CreateImpulseEE(const float3 * __restrict__ positions, const float3 * __restrict__ velocities,
	const Physics::PrimitiveContact & contact,
	uint32_t * __restrict__ impulseIDs, float3 * __restrict__ impulseValues, bool * __restrict__ impulseFlags, const int myImpulseStart, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{

	float3 cv1 = contact.w1 * velocities[contact.v1] + contact.w2 * velocities[contact.v2];
	float3 cv2 = contact.w3 * velocities[contact.v3] + contact.w4 * velocities[contact.v4];
	float3 vr = cv1 + cv2;

	_CreateImpulse(positions, contact, vr, impulseIDs, impulseValues, impulseFlags, myImpulseStart, stiffness, vertexMass, timeStep, thickness);
}

__device__ void DeformableUtils::_CreateImpulse(const float3 * __restrict__ positions, const Physics::PrimitiveContact & contact, const float3 & vr, uint32_t *__restrict__ impulseIDs,
	float3 *__restrict__ impulseValues, bool * __restrict__ impulseFlags, const int myImpulseStart, const float stiffness, const float vertexMass, const float timeStep, const float thickness)
{

	float d = thickness - CudaUtils::dot(contact.w1 * positions[contact.v1] + contact.w2 * positions[contact.v2] + contact.w3 * positions[contact.v3] + contact.w4 * positions[contact.v4], contact.n);
	float vn = CudaUtils::dot(vr, contact.n);

	float dts = (0.1f * d) / timeStep;
	float i = 0.f;

	if (vn < dts)
		i = -min(timeStep * stiffness * d, vertexMass * (dts - vn));

	if (vn < 0)
		i += vertexMass * vn * 0.5f;// *0.25f;

	//if (i == 0.f)
	//{
	//	impulseFlags[myImpulseStart] = false;
	//	impulseFlags[myImpulseStart + 1] = false;
	//	impulseFlags[myImpulseStart + 2] = false;
	//	impulseFlags[myImpulseStart + 3] = false;
	//	return;
	//}


	i = CudaUtils::clamp(i, -vertexMass * 0.1f * thickness, vertexMass * 0.1f * thickness);

	float iPrime = (2 * i) / (contact.w1 * contact.w1 + contact.w2 * contact.w2 + contact.w3 * contact.w3 + contact.w4 * contact.w4);

	float3 iPrimeVec = iPrime * contact.n;


	
	impulseIDs[myImpulseStart] = contact.v1;
	impulseIDs[myImpulseStart + 1] = contact.v2;
	impulseIDs[myImpulseStart + 2] = contact.v3;
	impulseIDs[myImpulseStart + 3] = contact.v4;


	impulseValues[myImpulseStart] = contact.w1 * iPrimeVec;
	impulseValues[myImpulseStart + 1] = contact.w2 * iPrimeVec;
	impulseValues[myImpulseStart + 2] = contact.w3 * iPrimeVec;
	impulseValues[myImpulseStart + 3] = contact.w4 * iPrimeVec;


	//impulseFlags[myImpulseStart] = contact.w1 != 0.f;
	//impulseFlags[myImpulseStart + 1] = contact.w2 != 0.f;
	//impulseFlags[myImpulseStart + 2] = contact.w3 != 0.f;
	//impulseFlags[myImpulseStart + 3] = contact.w4 != 0.f;
}

__global__ void DeformableUtils::AccumulateImpulses(const int impulseRunCount, const uint32_t *__restrict__ impulseRLEUniques,
	const int *__restrict__ impulseRLEPrefixSums,
	const float3 * __restrict__ impulseValues, const uint64_t impulseValuesSize, float3 *__restrict__ accumulatedImpulses)
{
	int id = CudaUtils::MyID();
	if (id >= impulseRunCount)
		return;

	int myAccImpulseID = impulseRLEUniques[id];

	int myStart = impulseRLEPrefixSums[id];
	int myEnd = id == impulseRunCount - 1 ? impulseValuesSize : impulseRLEPrefixSums[id + 1];

	int denom = myEnd - myStart;

	float3 acc = make_float3(0.f, 0.f, 0.f);

	for (int i = myStart; i < myEnd; ++i)
	{
		acc += impulseValues[i];
	}

	accumulatedImpulses[myAccImpulseID] = acc / denom;
}

void DeformableUtils::ApplyImpulses(thrust::device_vector<float3>& positions, const thrust::device_vector<float3> &prevPositions,
	thrust::device_vector<float3>& velocities,
	const thrust::device_vector<float3>& impulses, const thrust::device_vector<bool>& fixedVerts, const float vertexMass, const float timeStep)
{
	const int numBlocks = (positions.size() + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ApplyImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > ((int)positions.size(), thrust::raw_pointer_cast(&positions[0]),
		thrust::raw_pointer_cast(&prevPositions[0]),
		thrust::raw_pointer_cast(&velocities[0]), thrust::raw_pointer_cast(&impulses[0]), thrust::raw_pointer_cast(&fixedVerts[0]), vertexMass, timeStep);

}

__global__ void DeformableUtils::ApplyImpulses(const int particleCount, float3 *__restrict__ positions, const float3 *__restrict__ prevPositions,
	float3 *__restrict__ velocities,
	const float3 *__restrict__ impulses, const bool * __restrict__ fixedVerts, const float vertexMass, const float timeStep)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	if (fixedVerts[id])
		return;

	/*if (CudaUtils::len(impulses[id]) != 0.f)
	printf("[%d] accImp: (%f, %f, %f), \n", id, impulses[id].x / vertexMass, impulses[id].z / vertexMass, impulses[id].z / vertexMass);*/

	/*float3 newVel = impulses[id] / vertexMass;

	float3 velDir = newVel - velocities[id];

	velocities[id] += CudaUtils::normalize(velDir) * (min(0.1f * 0.25f / 20.f, CudaUtils::len(velDir)));*/

	velocities[id] += impulses[id] / vertexMass;
	positions[id] = prevPositions[id] + velocities[id] * timeStep;
}






//void DeformableUtils::CreateImpulses(const thrust::device_vector<float3> &positions, const thrust::device_vector<float3> &velocities,
//	const thrust::device_vector<Physics::PrimitiveContact> &vfContacts, const uint64_t vfContactsSize,
//	const thrust::device_vector<Physics::PrimitiveContact> &eeContacts, const uint64_t eeContactsSize,
//	thrust::device_vector<uint32_t> &impulseIDs, thrust::device_vector<float3> &impulseValues, thrust::device_vector<bool> &impulseFlags, uint64_t &impulsesSize,
//	thrust::device_vector<uint32_t> &impulseRLEUniques, thrust::device_vector<int> &impulseRLECounts, int &impulseRunCount,
//	thrust::device_vector<float3> &accumulatedImpulses, const float stiffness, const float vertexMass, const float timeStep, const float thickness,
//	void *&tempStorage, uint64_t &tempStorageSize)
//{
//
//	cudaMemset(thrust::raw_pointer_cast(&accumulatedImpulses[0]), 0, accumulatedImpulses.size() * sizeof(float3));
//
//	uint64_t totalImpulses = 4 * (vfContactsSize + eeContactsSize);
//
//	if (impulsesSize < totalImpulses)
//	{
//		impulseIDs.resize(totalImpulses);
//		impulseValues.resize(totalImpulses);
//		impulseFlags.resize(totalImpulses);
//		impulseRLEUniques.resize(totalImpulses);
//		impulseRLECounts.resize(totalImpulses);
//		impulsesSize = totalImpulses;
//	}
//
//
//	if (totalImpulses == 0)
//		return;
//
//
//	int numBlocks = (vfContactsSize + eeContactsSize + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;
//
//	_CreateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (thrust::raw_pointer_cast(&positions[0]), thrust::raw_pointer_cast(&velocities[0]),
//		thrust::raw_pointer_cast(&vfContacts[0]), vfContactsSize,
//		thrust::raw_pointer_cast(&eeContacts[0]), eeContactsSize,
//		thrust::raw_pointer_cast(&impulseIDs[0]),
//		thrust::raw_pointer_cast(&impulseValues[0]), stiffness, vertexMass, timeStep, thickness);
//
//
//
//
//
//
//	CubWrap::SortByKey(impulseIDs, impulseValues, totalImpulses, tempStorage, tempStorageSize);
//
//	thrust::host_vector<uint32_t> hids(impulseIDs.begin(), impulseIDs.begin() + totalImpulses);
//	thrust::host_vector<float3> hvals(impulseValues.begin(), impulseValues.begin() + totalImpulses);
//
//	for (int i = 0; i < totalImpulses - 1; ++i)
//	{
//		if (hids[i] > hids[i + 1])
//			printf("[%d] impulse: %d > %d -> (%f, %f, %f)\n", i, hids[i], hids[i + 1], hvals[i].x, hvals[i].y, hvals[i].z);
//	}
//
//	CubWrap::RLE(impulseIDs, totalImpulses, impulseRLEUniques, impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);
//
//	thrust::host_vector<uint32_t> huniq(impulseRLEUniques.begin(), impulseRLEUniques.begin() + impulseRunCount);
//	thrust::host_vector<int> hcount(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);
//
//	//
//	//for (int i = 0; i < impulseRunCount; ++i)
//	//{
//	//	printf("RLE: %d -> %d\n", huniq[i], hcount[i]);
//	//}
//
//	CubWrap::ExclusiveSumInPlace(impulseRLECounts, impulseRunCount, tempStorage, tempStorageSize);
//
//	//thrust::host_vector<int> hPref(impulseRLECounts.begin(), impulseRLECounts.begin() + impulseRunCount);
//	//for (int i = 0; i < impulseRunCount; ++i)
//	//{
//	//	printf("Prefix: %d - %d\n", hcount[i], hPref[i]);
//	//}
//
//	numBlocks = (impulseRunCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;
//
//	if (impulseRunCount > accumulatedImpulses.size())
//	{
//		printf("EROAREOARE\n");
//
//		for (int i = 0; i < impulseRunCount; ++i)
//		{
//			printf("[%d], RLE: %d -> %d\n", i, huniq[i], hcount[i]);
//		}
//
//		//for (int i = 0; i < totalImpulses - 1; ++i)
//		//{
//		//	if (hids[i] > hids[i + 1])
//		//		printf("impulse: %d -> (%f, %f, %f)\n", hids[i], hvals[i].x, hvals[i].y, hvals[i].z);
//		//}
//
//		int z = 15;
//	}
//	AccumulateImpulses << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (impulseRunCount, thrust::raw_pointer_cast(&impulseRLEUniques[0]),
//		thrust::raw_pointer_cast(&impulseRLECounts[0]), thrust::raw_pointer_cast(&impulseValues[0]), totalImpulses, thrust::raw_pointer_cast(&accumulatedImpulses[0]));
//
//	thrust::host_vector<float3> hacc(accumulatedImpulses);
//
//	//for (int i = 0; i < hacc.size(); ++i)
//	//{
//	//	if (hacc[i].x == 0 && hacc[i].y == 0 && hacc[i].z == 0)
//	//		continue;
//	//	printf("[%d] final imp: (%f, %f, %f)\n", i, hacc[i].x, hacc[i].y, hacc[i].z);
//	//}
//}