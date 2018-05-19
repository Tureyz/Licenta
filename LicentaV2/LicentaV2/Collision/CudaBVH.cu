#include "CudaBVH.cuh"

#include "../Core/CudaUtils.cuh"
#include <thrust/host_vector.h>
#include <unordered_map>
#include <set>


void CudaBVH::GetAABBCollisions(const thrust::device_vector<int>& lefts, const thrust::device_vector<int>& rights, const thrust::device_vector<float3>& AABBMins,
	const thrust::device_vector<float3>& AABBMaxs, const thrust::device_vector<int>& rmlls, const thrust::device_vector<int>& rmlrs,
	thrust::device_vector<Physics::AABBCollision> &collisions, const int chunkSize, const uint64_t timestamp)
{
	int leafCount = lefts.size() + 1;
	int numBlocks = (leafCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_GetAABBCollisions << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (leafCount, thrust::raw_pointer_cast(&lefts[0]), thrust::raw_pointer_cast(&rights[0]),
		thrust::raw_pointer_cast(&AABBMins[0]), thrust::raw_pointer_cast(&AABBMaxs[0]), thrust::raw_pointer_cast(&rmlls[0]), thrust::raw_pointer_cast(&rmlrs[0]),
		thrust::raw_pointer_cast(&collisions[0]), chunkSize, timestamp);

	//printf("Done with AABB cols\n");
}

__global__ void CudaBVH::_GetAABBCollisions(const int leafCount, const int *__restrict__ lefts, const int *__restrict__ rights, const float3 *__restrict__ aabbMins,
	const float3 *__restrict__ aabbMaxs, const int *__restrict__ rmlls, const int *__restrict__ rmlrs,
	Physics::AABBCollision * __restrict__ collisions, const int chunkSize, const uint64_t timestamp)
{
	int id = CudaUtils::MyID();
	if (id >= leafCount)
		return;


	const int internalNodeCount = leafCount - 1;
	const int myLeafID = internalNodeCount + id;
	const int myChunk = chunkSize * id;
	int crtCol = 0;


	int nodeStack[64];
	nodeStack[0] = -1;
	int stackTop = 1;
	

	int crtNode = 0; // root

	do
	{
		//printf("[%d] crtNode: %d\n", id, crtNode);

		int left = lefts[crtNode];
		int right = rights[crtNode];

		bool overlapL = false;
		bool overlapR = false;

		if (rmlls[crtNode] > myLeafID)
		{
			if (CudaUtils::AABBOverlap(aabbMins[myLeafID], aabbMaxs[myLeafID], aabbMins[left], aabbMaxs[left]))
				overlapL = true;
		}

		if (rmlrs[crtNode] > myLeafID)
		{
			if (CudaUtils::AABBOverlap(aabbMins[myLeafID], aabbMaxs[myLeafID], aabbMins[right], aabbMaxs[right]))
				overlapR = true;
		}

		if (overlapL && left >= internalNodeCount)
		{
			if (crtCol < chunkSize)
			{
				collisions[myChunk + crtCol++] = Physics::AABBCollision(id, left - internalNodeCount, timestamp);
				//printf("[%d] Collision between %d and %d\n", id, myLeafID, left - internalNodeCount);
			}
			else
			{
				//printf("[%d] Out of space, myLeaf: %d, lleaf #: %d, crtCol: %d\n", id, myLeafID, left - internalNodeCount, crtCol++);
			}
		}

		if (overlapR && right >= internalNodeCount)
		{
			if (crtCol < chunkSize)
			{
				collisions[myChunk + crtCol++] = Physics::AABBCollision(id, right - internalNodeCount, timestamp);
				//printf("[%d] Collision between %d and %d\n", id, myLeafID, right - internalNodeCount);
			}
			else
			{
				//printf("[%d] Out of space, myLeaf: %d, rleaf #: %d, crtCol: %d\n", id, myLeafID, right - internalNodeCount, crtCol++);
			}
		}

		bool traverseL = (overlapL && left < internalNodeCount);
		bool traverseR = (overlapR && right < internalNodeCount);

		if (!traverseL && !traverseR)
		{
			crtNode = nodeStack[--stackTop];
		}
		else
		{
			crtNode = traverseL ? left : right;
			if (traverseL && traverseR)
				nodeStack[stackTop++] = right;
		}


	} while (stackTop != 0);

	//AABBCollisionSizes[id] = crtCol;
}

void CudaBVH::ComputeTreeAABBs(const thrust::device_vector<int>& lefts, const thrust::device_vector<int>& rights, const thrust::device_vector<int>& parents,
	thrust::device_vector<int>& nodesVisited, thrust::device_vector<float3> &AABBMins, thrust::device_vector<float3> &AABBMaxs,
	thrust::device_vector<int> &rmlls, thrust::device_vector<int> &rmlrs)
{
	int leafCount = lefts.size() + 1;
	int numBlocks = (leafCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ComputeTreeAABBs <<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(leafCount, thrust::raw_pointer_cast(&lefts[0]), thrust::raw_pointer_cast(&rights[0]),
		thrust::raw_pointer_cast(&parents[0]), thrust::raw_pointer_cast(&nodesVisited[0]), thrust::raw_pointer_cast(&AABBMins[0]), thrust::raw_pointer_cast(&AABBMaxs[0]),
		thrust::raw_pointer_cast(&rmlls[0]), thrust::raw_pointer_cast(&rmlrs[0]));

}

__global__ void CudaBVH::ComputeTreeAABBs(const int leafNodeCount, const int *__restrict__ lefts, const int *__restrict__ rights,
	const int *__restrict__ parents, int *__restrict__ nodesVisited, float3 *__restrict__ AABBMins, float3 *__restrict__ AABBMaxs,
	int * __restrict__ rmlls, int * __restrict__ rmlrs)
{
	int id = CudaUtils::MyID();
	if (id >= leafNodeCount)
		return;


	for (int pID = parents[leafNodeCount - 1 + id]; atomicExch(&nodesVisited[pID], 1) == 1; pID = parents[pID])
	{
		//printf("[%d] Parent: %d\n", pID, parents[pID]);
		AABBMins[pID] = CudaUtils::minf3(AABBMins[lefts[pID]], AABBMins[rights[pID]]);
		AABBMaxs[pID] = CudaUtils::maxf3(AABBMaxs[lefts[pID]], AABBMaxs[rights[pID]]);

		rmlls[pID] = lefts[pID] >= leafNodeCount - 1 ? lefts[pID] : max(rmlls[lefts[pID]], rmlrs[lefts[pID]]);
		rmlrs[pID] = rights[pID] >= leafNodeCount - 1 ? rights[pID] : max(rmlls[rights[pID]], rmlrs[rights[pID]]);

		if (pID == 0)
			return;
	}
}


void CudaBVH::GenerateBVH2(const thrust::device_vector<uint64_t>& sortedMortons, thrust::device_vector<int>& lefts, thrust::device_vector<int>& rights,
	thrust::device_vector<int>& parents, thrust::device_vector<int>& nodesVisited)
{
	const int internalNodeCount = sortedMortons.size() - 1;

	int numBlocks = (internalNodeCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_GenerateBVH2 << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(internalNodeCount, thrust::raw_pointer_cast(&sortedMortons[0]),
		thrust::raw_pointer_cast(&lefts[0]), thrust::raw_pointer_cast(&rights[0]), thrust::raw_pointer_cast(&parents[0]), thrust::raw_pointer_cast(&nodesVisited[0]));
}

__global__ void CudaBVH::_GenerateBVH2(const int internalNodeCount, const uint64_t *__restrict__ sortedMortons, int *__restrict__ lefts, int *__restrict__ rights, int *__restrict__ parents, int *__restrict__ nodesVisited)
{
	int id = CudaUtils::MyID();
	if (id >= internalNodeCount)
		return;

	nodesVisited[id] = 0;

	int2 range = FindRange(sortedMortons, internalNodeCount, id);
	int first = range.x;
	int last = range.y;

	int split = FindSplit(sortedMortons, first, last);

	int leftID = split == first ? internalNodeCount + split : split;
	int rightID = split + 1 == last ? internalNodeCount + split + 1 : split + 1;

	//printf("[%d] First: %d, Last: %d, Split: %d\n", id, first, last, split);
	//printf("[%d] Left: %d, Right: %d\n", id, leftID, rightID);

	lefts[id] = leftID;
	rights[id] = rightID;

	parents[leftID] = id;
	parents[rightID] = id;
}

__device__ int2 CudaBVH::FindRange(const uint64_t *__restrict__ sortedMortons, const int count, const int id)
{
	int d = CudaUtils::sgn(CudaUtils::Delta(sortedMortons, id, id + 1, count) - CudaUtils::Delta(sortedMortons, id, id - 1, count));
	int delMin = CudaUtils::Delta(sortedMortons, id, id - d, count);

	int lMax = 2;

	while (CudaUtils::Delta(sortedMortons, id, id + lMax * d, count) > delMin)
		lMax <<= 1;


	//printf("\t[%d] d: %d, dLeft: %d, dRight: %d\n", id, d, CudaUtils::Delta(sortedMortons, id, id - 1, count), CudaUtils::Delta(sortedMortons, id, id + 1, count));
	int l = 0;
	for (int t = lMax / 2; t >= 1; t /= 2)
	{
		if (CudaUtils::Delta(sortedMortons, id, id + (l + t) * d, count) > delMin)
			l += t;
	}

	int j = id + l * d;

	return id <= j ? make_int2(id, j) : make_int2(j, id);
}

__device__ int CudaBVH::FindSplit(const uint64_t *__restrict__ sortedMortons, const int first, const int last)
{
	uint64_t firstCode = sortedMortons[first];
	uint64_t lastCode = sortedMortons[last];

	if (firstCode == lastCode)
		return (first + last) >> 1;

	int comPref = __clzll(firstCode ^ lastCode);

	int split = first;
	int step = last - first;

	do
	{
		step = (step + 1) >> 1;
		int newSplit = split + step;

		if (newSplit < last)
		{
			uint64_t splitCode = sortedMortons[newSplit];
			int splitPref = __clzll(firstCode ^ splitCode);
			if (splitPref > comPref)
				split = newSplit;
		}
	} while (step > 1);

	return split;
}

void CudaBVH::BVHTestUnit(thrust::device_vector<uint64_t>& sortedMortons, const thrust::device_vector<int>& lefts, const thrust::device_vector<int>& rights, const thrust::device_vector<int>& parents, const thrust::device_vector<int>& nodesVisited, const thrust::device_vector<float3>& AABBMins, const thrust::device_vector<float3>& AABBMaxs)
{
	thrust::host_vector<int> hLefts = lefts;
	thrust::host_vector<int> hRights = rights;
	thrust::host_vector<int> hParents = parents;
	thrust::host_vector<int> hNodesVisited = nodesVisited;
	thrust::host_vector<float3> hAABBmins = AABBMins;
	thrust::host_vector<float3> hAABBmaxs = AABBMaxs;
	thrust::host_vector<uint64_t> hMortons = sortedMortons;

	const int leafCount = hMortons.size();
	const int internalCount = leafCount - 1;


	std::unordered_map<uint64_t, int> mortonDuplicates;

	for (int i = 0; i < hMortons.size() - 1; ++i)
	{
		//std::cout << "mortons[" << i << "]: " << hMortons[i] << std::endl;
		mortonDuplicates[hMortons[i]]++;

		if (hMortons[i] > hMortons[i + 1])
		{			
			std::cout << "Morton codes NOT OK" << std::endl;			
		}
	}

	for (auto kvPair : mortonDuplicates)
	{
		if (kvPair.second != 1)
		{
		//	std::cout << "DUPLICATE MORTONS\n";
			break;
		}
	}

	std::unordered_map<int, int> parentLinkCount;

	for (auto parent : hParents)
	{
		parentLinkCount[parent]++;
	}

	for (auto kvPair : parentLinkCount)
	{
		if ((kvPair.first == 0 && kvPair.second != 3) || (kvPair.first > 0 && kvPair.second != 2))
		{
			std::cout << "Parent links NOT OK" << std::endl;
			break;
		}

		if (kvPair.first >= internalCount)
		{
			std::cout << "Leaves appear as parents!" << std::endl;
			break;
		}
	}

	std::unordered_map<int, int> leftLinkCount;

	for (auto left : hLefts)
	{
		leftLinkCount[left]++;
	}


	for (auto kvPair : leftLinkCount)
	{
		if ((kvPair.first == 0 && kvPair.second != 0) || (kvPair.first > 0 && kvPair.second != 1))
		{
			std::cout << "Left links NOT OK" << std::endl;
			break;
		}
	}

	std::unordered_map<int, int> rightLinkCount;

	for (auto right : hRights)
	{
		rightLinkCount[right]++;
	}

	for (auto kvPair : rightLinkCount)
	{
		if ((kvPair.first == 0 && kvPair.second != 0) || (kvPair.first > 0 && kvPair.second != 1))
		{
			std::cout << "Right links NOT OK" << std::endl;
			break;
		}
	}

	for (auto kvPair : leftLinkCount)
	{
		if (rightLinkCount.count(kvPair.first))
		{
			std::cout << "Links NOT OK" << std::endl;
			break;
		}
	}

	std::set<int> visitedNodes;

	for (int i = 0; i < hNodesVisited.size(); ++i)
	{
		if (hNodesVisited[i] == 1)
		{
			visitedNodes.insert(i);
		}
	}

	//if (visitedNodes.size() != hNodesVisited.size())
	//{
	//	std::cout << "Nodes visited NOT OK" << std::endl;		
	//}


	for (auto aabbMin : hAABBmins)
	{
		if (aabbMin != aabbMin)
		{
			std::cout << "AABB mins NOT OK" << std::endl;
		}
	}

	for (auto aabbMax : hAABBmaxs)
	{
		if (aabbMax != aabbMax)
		{
			std::cout << "AABB maxs NOT OK" << std::endl;
		}
	}

}

void CudaBVH::PrintTree(const thrust::device_vector<int>& lefts, const thrust::device_vector<int>& rights, const thrust::device_vector<int>& parents,
	const thrust::device_vector<int> &rmlls, const thrust::device_vector<int> &rmlrs)
{
	PrintTree<<<1, 1>>>(lefts.size() + 1, thrust::raw_pointer_cast(&lefts[0]), thrust::raw_pointer_cast(&rights[0]),
		thrust::raw_pointer_cast(&parents[0]), thrust::raw_pointer_cast(&rmlls[0]), thrust::raw_pointer_cast(&rmlrs[0]));
}

__global__ void CudaBVH::PrintTree(const int leafNodeCount, const int *__restrict__ lefts, const int *__restrict__ rights, const int *__restrict__ parents,
	const int * __restrict__ rmlls, const int * __restrict__ rmlrs)
{
	printf("Internal Nodes:\n");
	for (int i = 0; i < leafNodeCount - 1; ++i)
	{
		printf("[%d] Left: %d, Right: %d, Parent: %d\n", i, lefts[i], rights[i], parents[i]);
	}

	printf("Leaves:\n");

	for (int i = 0; i < leafNodeCount; ++i)
	{
		printf("[%d] Parent: %d\n", i + leafNodeCount - 1, parents[i + leafNodeCount - 1]);
	}

	printf("RMs:\n");

	for (int i = 0; i < leafNodeCount - 1; ++i)
	{
		printf("[%d] RML: %d, RMR: %d\n", i, rmlls[i], rmlrs[i]);
	}
}














//void CudaBVH::GenerateBVH(const thrust::device_vector<unsigned int>& sortedMortons, 
//	thrust::device_vector<int>& lefts, thrust::device_vector<int>& rights, thrust::device_vector<int>& parents, thrust::device_vector<int> &nodesVisited)
//{
//
//	const int internalNodeCount = sortedMortons.size() - 1;
//
//	int numBlocks = (internalNodeCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;
//
//	_GenerateBVH<<<numBlocks, CudaUtils::THREADS_PER_BLOCK>>>(internalNodeCount, thrust::raw_pointer_cast(&sortedMortons[0]), 
//		thrust::raw_pointer_cast(&lefts[0]), thrust::raw_pointer_cast(&rights[0]), thrust::raw_pointer_cast(&parents[0]), thrust::raw_pointer_cast(&nodesVisited[0]));
//}




//__global__ void CudaBVH::_GenerateBVH(const int internalNodeCount, const unsigned int * __restrict__ sortedMortons,
//	int * __restrict__ lefts, int * __restrict__ rights, int * __restrict__ parents, int * __restrict__ nodesVisited)
//{
//	int id = CudaUtils::MyID();
//	if (id >= internalNodeCount)
//		return;
//
//	nodesVisited[id] = 0;
//
//	int d = CudaUtils::sgn(CudaUtils::ComPref(sortedMortons[id], sortedMortons[id + 1], id, id + 1, internalNodeCount) -
//		CudaUtils::ComPref(sortedMortons[id], sortedMortons[id - 1], id, id - 1, internalNodeCount));
//
//	int2 range = FindRange(sortedMortons, id, d, internalNodeCount);
//
//	int first = range.x;
//	int last = range.y;
//
//	int j = id + first * d;
//
//
//	int split = FindSplit(sortedMortons, id, d, j, first, internalNodeCount);
//
//
//	int leftID = min(id, j) == split ? internalNodeCount + split : split;
//	int rightID = max(id, j) == split + 1 ? internalNodeCount + split + 1 : split + 1;
//
//
//	//printf("[%d] First: %d, Last: %d, Split: %d\n", id, id, j, split);
//	printf("[%d] Left: %d, Right: %d\n", id, leftID, rightID);
//
//	lefts[id] = leftID;
//	rights[id] = rightID;
//
//	parents[leftID] = id;
//	parents[rightID] = id;
//}
//
//__device__ int CudaBVH::BinarySearch(const int start, const int del, const unsigned int *__restrict__ sortedMortons, const int id, const int d, const int internalNodeCount)
//{
//	int l = 0;
//
//	for (int t = ceilf(start / 2); t >= 1; t = ceilf(t >> 1))
//	{
//		if (CudaUtils::ComPref(sortedMortons[id], sortedMortons[id + (l + t) * d], id, id + (l + t) * d, internalNodeCount) > del)
//			l += t;
//	}
//
//	return l;
//}
//
//__device__ int2 CudaBVH::FindRange(const unsigned int *__restrict__ sortedMortons, const int id, const int d, const int internalNodeCount)
//{
//	int delMin = CudaUtils::ComPref(sortedMortons[id], sortedMortons[id - d], id, id - d, internalNodeCount);
//	int lMax = 2;
//
//	while (CudaUtils::ComPref(sortedMortons[id], sortedMortons[id + lMax * d], id, id + lMax * d, internalNodeCount) > delMin)
//		lMax <<= 1;
//
//	int l = BinarySearch(lMax, delMin, sortedMortons, id, d, internalNodeCount);
//
//
//
//	return make_int2(l, lMax);
//}
//
//__device__ int CudaBVH::FindSplit(const unsigned int *__restrict__ sortedMortons, const int id, const int d, const int j, const int l, const int internalNodeCount)
//{
//	int delNode = CudaUtils::ComPref(sortedMortons[id], sortedMortons[j], id, j, internalNodeCount);
//
//	int s = BinarySearch(l, delNode, sortedMortons, id, d, internalNodeCount);
//
//	return id + s * d + min(d, 0);
//}