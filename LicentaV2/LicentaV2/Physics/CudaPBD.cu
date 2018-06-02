#include "CudaPBD.cuh"

#include "../Dependencies/glm/glm.hpp"

#include "../Core/CubWrappers.cuh"
#include <unordered_map>
#include "../Core/CudaUtils.cuh"

#include <thrust/host_vector.h>

namespace std
{
	template <> struct equal_to<int2>
	{
		inline bool operator()(const int2 &l, const int2 &r) const
		{
			return (l.x == r.x && l.y == r.y) || (l.x == r.y && l.y == r.x);
		}
	};

	template <> struct hash<int2>
	{
		inline size_t operator()(const int2 &v) const {
			std::hash<int> hasher;
			return hasher(v.x) ^ hasher(v.y);
		}
	};
}


__device__ const float EPS = 0.00000001f;

#define PI 3.14159265359f

void CudaPBD::CudaPBD::Init(const Physics::ClothParams parentParams, const std::vector<Rendering::VertexFormat> &verts, const std::vector<unsigned int>  &indices)
{
	m_parentParams = parentParams;

	m_particleCount = m_parentParams.dims.x * m_parentParams.dims.y;



	thrust::host_vector<bool> hfv = m_parentParams.fixedVerts;

	thrust::host_vector<int> fixedVerts;

	for (int i = 0; i < hfv.size(); ++i)
	{
		if (hfv[i])
		{
			fixedVerts.push_back(i);
		}
	}

	thrust::host_vector<float> l0s(m_particleCount * fixedVerts.size());

	for (int i = 0; i < m_particleCount; ++i)
	{
		for (int j = 0; j < fixedVerts.size(); ++j)
		{
			l0s[i * fixedVerts.size() + j] = glm::distance(verts[i].m_position, verts[fixedVerts[j]].m_position) * (1.f + m_parentParams.strainLimit);
		}
	}

	m_LRAConstraints.fixedPosID = fixedVerts;
	m_LRAConstraints.fixedVertCount = fixedVerts.size();
	m_LRAConstraints.l0 = l0s;
	m_LRAConstraints.flag.resize(m_particleCount * m_LRAConstraints.fixedVertCount);

	std::unordered_map<int2, int3> edgeTriangleMap;

	//std::vector<std::vector<bool>> takenEdges(m_parentParams.dims.x, std::vector<bool>(m_parentParams.dims.y));

	thrust::host_vector<int> hsid1;
	thrust::host_vector<int> hsid2;
	thrust::host_vector<float> hsl0;

	thrust::host_vector<int> hbid1;
	thrust::host_vector<int> hbid2;
	thrust::host_vector<int> hbid3;
	thrust::host_vector<int> hbid4;
	thrust::host_vector<float> hbphi0;

	for (int i = 0; i < indices.size(); i += 3)
	{
		int triv1 = indices[i];
		int triv2 = indices[i + 1];
		int triv3 = indices[i + 2];

		int3 triangle = make_int3(triv1, triv2, triv3);

		for (int j = 0; j < 3; ++j)
		{

			int id1 = indices[i + (j % 3)];
			int id2 = indices[i + ((j + 1) % 3)];

			int2 edge = make_int2(id1, id2);
			auto found = edgeTriangleMap.find(edge);

			if (found != edgeTriangleMap.end())
			{
				CreateStaticBendConstraint((*found).second, triangle, verts, hbid1, hbid2, hbid3, hbid4, hbphi0);
				edgeTriangleMap.erase(edge);
			}
			else
			{
				edgeTriangleMap[edge] = triangle;
				CreateStaticStretchConstraint(id1, id2, verts, hsid1, hsid2, hsl0);
			}
		}
	}

	std::vector<int> counts(m_particleCount, 0);
	std::vector<int> offsets(m_particleCount, 0);
	thrust::host_vector<int> prefixSums(m_particleCount + 1, 0);

	int totalCount = 0;

	for (int i = 0; i < hsid1.size(); ++i)
	{
		totalCount += 2;
		counts[hsid1[i]]++;
		counts[hsid2[i]]++;
	}

	for (int i = 0; i < hbid1.size(); ++i)
	{
		totalCount += 4;
		counts[hbid1[i]]++;
		counts[hbid2[i]]++;
		counts[hbid3[i]]++;
		counts[hbid4[i]]++;
	}

	for (int i = 1; i < counts.size() + 1; ++i)
	{
		prefixSums[i] = prefixSums[i - 1] + counts[i - 1];
	}

	m_staticCorrectionPrefixSums = prefixSums;

	counts = std::vector<int>(m_particleCount, 0);


	thrust::host_vector<int> hsid1o(hsid1.size(), -1);
	thrust::host_vector<int> hsid2o(hsid2.size(), -1);

	for (int i = 0; i < hsid1.size(); ++i)
	{
		hsid1o[i] = prefixSums[hsid1[i]] + (counts[hsid1[i]]++);
		hsid2o[i] = prefixSums[hsid2[i]] + (counts[hsid2[i]]++);
	}

	thrust::host_vector<int> hbid1o(hbid1.size(), -1);
	thrust::host_vector<int> hbid2o(hbid2.size(), -1);
	thrust::host_vector<int> hbid3o(hbid3.size(), -1);
	thrust::host_vector<int> hbid4o(hbid4.size(), -1);

	for (int i = 0; i < hbid1.size(); ++i)
	{
		hbid1o[i] = prefixSums[hbid1[i]] + (counts[hbid1[i]]++);
		hbid2o[i] = prefixSums[hbid2[i]] + (counts[hbid2[i]]++);
		hbid3o[i] = prefixSums[hbid3[i]] + (counts[hbid3[i]]++);
		hbid4o[i] = prefixSums[hbid4[i]] + (counts[hbid4[i]]++);
	}



	//for (int i = 0; i < hsid1.size(); ++i)
	//{
	//	if (hsid1o[i] >= totalCount)
	//	{
	//		printf("hsid1o[%d] = %d\n", i, hsid1o[i]);
	//	}
	//	if (hsid2o[i] >= totalCount)
	//	{
	//		printf("hsid2o[%d] = %d\n", i, hsid2o[i]);
	//	}
	//}

	//for (int i = 0; i < hbid1.size(); ++i)
	//{
	//	if (hbid1o[i] >= totalCount)
	//	{
	//		printf("hbid1o[%d] = %d\n", i, hbid1o[i]);
	//	}
	//	if (hbid2o[i] >= totalCount)
	//	{
	//		printf("hbid2o[%d] = %d\n", i, hbid2o[i]);
	//	}
	//	if (hbid3o[i] >= totalCount)
	//	{
	//		printf("hbid3o[%d] = %d\n", i, hbid3o[i]);
	//	}
	//	if (hbid4o[i] >= totalCount)
	//	{
	//		printf("hbid4o[%d] = %d\n", i, hbid4o[i]);
	//	}
	//}

	m_stretchConstraints.id1 = hsid1;
	m_stretchConstraints.id2 = hsid2;
	m_stretchConstraints.id1Offset = hsid1o;
	m_stretchConstraints.id2Offset = hsid2o;
	m_stretchConstraints.l0 = hsl0;
	m_stretchConstraints.k = m_parentParams.kStretch;

	m_bendConstraints.id1 = hbid1;
	m_bendConstraints.id2 = hbid2;
	m_bendConstraints.id3 = hbid3;
	m_bendConstraints.id4 = hbid4;
	m_bendConstraints.id1Offset = hbid1o;
	m_bendConstraints.id2Offset = hbid2o;
	m_bendConstraints.id3Offset = hbid3o;
	m_bendConstraints.id4Offset = hbid4o;
	m_bendConstraints.phi0 = hbphi0;
	m_bendConstraints.k = m_parentParams.kBend;



	//m_staticCorrectionIDs.resize(totalCount);
	m_staticCorrectionValues.resize(totalCount);
	m_accumulatedCorrectionValues.resize(m_particleCount);
	m_accumulatedCorrectionCounts.resize(m_particleCount);
	m_aux1.resize(m_particleCount);
	m_aux2.resize(m_particleCount);


	m_groundConstraints.flag.resize(m_particleCount);
	m_groundConstraints.nc.resize(m_particleCount);
	m_groundConstraints.qc.resize(m_particleCount);

}

void CudaPBD::CudaPBD::CreateStaticStretchConstraint(const int id1, const int id2, const std::vector<Rendering::VertexFormat> &verts,
	thrust::host_vector<int> &sid1, thrust::host_vector<int> &sid2, thrust::host_vector<float> &sl0)
{
	sid1.push_back(id1);
	sid2.push_back(id2);
	sl0.push_back(glm::distance(verts[id1].m_position, verts[id2].m_position));

	//m_stretchConstraints.k.push_back(m_parentParams.kStretch);
	//m_stretchConstraints.id1Offset.push_back(-1);
	//m_stretchConstraints.id2Offset.push_back(-1);
}

void CudaPBD::CudaPBD::CreateStaticBendConstraint(const int3 tri1, const int3 tri2, const std::vector<Rendering::VertexFormat> &verts,
	thrust::host_vector<int> &bid1, thrust::host_vector<int> &bid2, thrust::host_vector<int> &bid3,
	thrust::host_vector<int> &bid4,
	thrust::host_vector<float> &bphi0)
{
	int p1 = -1, p2 = -1, p3 = -1, p4 = -1;

	int t1[3], t2[3];
	t1[0] = tri1.x;
	t1[1] = tri1.y;
	t1[2] = tri1.z;

	t2[0] = tri2.x;
	t2[1] = tri2.y;
	t2[2] = tri2.z;
	
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (t1[i] == t2[j])
			{
				p1 = t1[i];
				break;
			}
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (t1[i] == t2[j] && t1[i] != p1)
			{
				p2 = t1[i];
				break;
			}
		}
	}


	for (int i = 0; i < 3; ++i)
	{
		if (t1[i] != p1 && t1[i] != p2)
		{
			p3 = t1[i];
			break;
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		if (t2[i] != p1 && t2[i] != p2)
		{
			p4 = t2[i];
			break;
		}
	}

	glm::vec3 p21Vec = verts[p2].m_position - verts[p1].m_position;
	glm::vec3 p31Vec = verts[p3].m_position - verts[p1].m_position;
	glm::vec3 p41Vec = verts[p4].m_position - verts[p1].m_position;

	bid1.push_back(p1);
	bid2.push_back(p2);
	bid3.push_back(p3);
	bid4.push_back(p4);



	//printf("%f\n", glm::dot(glm::normalize(glm::cross(p21Vec, p31Vec)), glm::normalize(glm::cross(p21Vec, p41Vec))));
	/*m_bendConstraints.id1Offset.push_back(-1);
	m_bendConstraints.id2Offset.push_back(-1);
	m_bendConstraints.id3Offset.push_back(-1);
	m_bendConstraints.id4Offset.push_back(-1);*/

	//m_bendConstraints.k.push_back(m_parentParams.kBend);
	float phi0 = std::acos(glm::dot(glm::normalize(glm::cross(p21Vec, p31Vec)), glm::normalize(glm::cross(p21Vec, p41Vec))));

	bphi0.push_back(PI);

}

void CudaPBD::CudaPBD::CreateGroundConstraints(const thrust::device_vector<float3>& positions,
	const thrust::device_vector<float3>& prevPositions, const thrust::device_vector<float>& invMasses)
{
	const int particleCount = positions.size();

	const int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_CreateGroundConstraints << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), cu::raw(prevPositions), cu::raw(invMasses), m_parentParams.worldMin.y,
		m_parentParams.thickness, cu::raw(m_groundConstraints.qc), cu::raw(m_groundConstraints.nc), cu::raw(m_groundConstraints.flag));
}

void CudaPBD::CudaPBD::CreateLRAConstraints(const thrust::device_vector<float3>& positions)
{
	
	const int numBlocks = (m_particleCount * m_LRAConstraints.fixedVertCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_CreateLRAConstraints << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_particleCount, m_LRAConstraints.fixedVertCount, cu::raw(positions), cu::raw(m_LRAConstraints.fixedPosID),
		cu::raw(m_LRAConstraints.l0), cu::raw(m_LRAConstraints.flag));
}



void CudaPBD::CudaPBD::DampVelocities(const thrust::device_vector<float3>& positions, const thrust::device_vector<float>& masses,
	thrust::device_vector<float3>& velocities, void *& tempStorage, uint64_t &tempStorageSize)
{

	const int particleCount = (const int)positions.size();
	const int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;


	float massSum = CubWrap::ReduceSum(masses, tempStorage, tempStorageSize);

	float vertexMass;

	cudaMemcpy(&vertexMass, cu::raw(masses), sizeof(float), cudaMemcpyDeviceToHost);

	//const float massSum = vertexMass * masses.size();


	ComputeXMVMs << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), cu::raw(velocities), cu::raw(masses), cu::raw(m_aux1), cu::raw(m_aux2));
	cudaCheckError();
	const float3 ximiSum = CubWrap::ReduceSum(m_aux1, tempStorage, tempStorageSize);
	const float3 vimiSum = CubWrap::ReduceSum(m_aux2, tempStorage, tempStorageSize);

	const float3 xcm = ximiSum / massSum;
	const float3 vcm = vimiSum / massSum;


	ComputeL << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), xcm, cu::raw(m_aux2), cu::raw(m_aux1));
	cudaCheckError();
	const float3 L = CubWrap::ReduceSum(m_aux1, tempStorage, tempStorageSize);

	ComputeIs << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), xcm, cu::raw(masses), cu::raw(m_aux1), cu::raw(m_aux2));
	cudaCheckError();
	const float3 i1 = CubWrap::ReduceSum(m_aux1, tempStorage, tempStorageSize);
	const float3 i2 = CubWrap::ReduceSum(m_aux2, tempStorage, tempStorageSize);

	const glm::mat3 I(glm::vec3(i1.y + i1.z, i2.x, i2.y),
		glm::vec3(i2.x, i1.x + i1.z, i2.z),
		glm::vec3(i2.y, i2.z, i1.x + i1.y));


	glm::vec3 omegaGLM = glm::inverse(I) * CudaUtils::MakeVec(L);

	const float3 omega = make_float3(omegaGLM.x, omegaGLM.y, omegaGLM.z);

	UpdateVels << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), omega, xcm, vcm, m_parentParams.kDamp, cu::raw(velocities));
	cudaCheckError();
}

void CudaPBD::CudaPBD::PBDStepExternal(thrust::device_vector<float3> &positions, thrust::device_vector<float3> &prevPositions, const thrust::device_vector<float> &masses,
	thrust::device_vector<float3> &velocities, void *&tempStorage, uint64_t &tempStorageSize)
{
	const int particleCount = (int) positions.size();
	const int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	ApplyExternalForces << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(velocities), m_parentParams.gravity, m_parentParams.timestep);
	cudaCheckError();
	DampVelocities(positions, masses, velocities, tempStorage, tempStorageSize);

	ExplicitEuler << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(prevPositions), cu::raw(velocities), cu::raw(positions), m_parentParams.timestep);
	cudaCheckError();
}

void CudaPBD::CudaPBD::PBDStepSolver(thrust::device_vector<float3>& positions, const thrust::device_vector<float3>& prevPositions,
	const thrust::device_vector<float>& invMasses,
	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts, const uint64_t & vfContactsSize,
	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts, const uint64_t & eeContactsSize,
	void *& tempStorage, uint64_t & tempStorageSize)
{
	cudaMemset(cu::raw(m_groundConstraints.flag), 0, m_groundConstraints.flag.size() * sizeof(bool));
	cudaMemset(cu::raw(m_LRAConstraints.flag), 0, m_LRAConstraints.flag.size() * sizeof(bool));

	CreateGroundConstraints(positions, prevPositions, invMasses);
	CreateLRAConstraints(positions);

	for (int i = 0; i < m_parentParams.solverIterations; ++i)
	{
		cudaMemset(cu::raw(m_accumulatedCorrectionCounts), 0, m_accumulatedCorrectionCounts.size() * sizeof(int));
		cudaMemset(cu::raw(m_accumulatedCorrectionValues), 0, m_accumulatedCorrectionValues.size() * sizeof(float3));

		ProjectGroundConstraints(positions, invMasses);
		ProjectLRAConstraints(positions, invMasses);
		ProjectStaticConstraints(positions, invMasses, m_parentParams.solverIterations);
		ProjectDynamicConstraints(positions, invMasses, m_parentParams.thickness, vfContacts, vfContactsSize, eeContacts, eeContactsSize, tempStorage, tempStorageSize);
		ApplyCorrections(positions);
	}
}

void CudaPBD::CudaPBD::PBDStepFinal(thrust::device_vector<float3>& positions,
	thrust::device_vector<float3>& prevPositions, thrust::device_vector<float3>& velocities,
	const thrust::device_vector<bool>& fixedVerts)
{
	FinalUpdate(positions, prevPositions, velocities, fixedVerts);
}

void CudaPBD::CudaPBD::ProjectDynamicConstraints(const thrust::device_vector<float3> &positions, const thrust::device_vector<float> &invMasses,
	const float thickness,
	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts,
	const uint64_t vfContactsSize,
	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts,
	const uint64_t eeContactsSize,
	void *& tempStorage, uint64_t & tempStorageSize)
{
	int totalVF = vfContactsSize * 4;
	int totalEE = eeContactsSize * 4;

	int totalCorrections = totalVF + totalEE;

	if (totalCorrections == 0)
		return;

	if (m_dynamicCorrectionsSize < totalCorrections)
	{
		m_dbDynamicCorrectionID.buffers[0].resize(totalCorrections);
		m_dbDynamicCorrectionID.buffers[1].resize(totalCorrections);
		m_dbDynamicCorrectionValues.buffers[0].resize(totalCorrections);
		m_dbDynamicCorrectionValues.buffers[1].resize(totalCorrections);
		m_dDynamicCorrectionRLEUniques.resize(totalCorrections);
		m_dDynamicCorrectionRLECounts.resize(totalCorrections);
		m_dynamicCorrectionsSize = totalCorrections;
	}



	int numBlocks = (totalCorrections + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_ProjectDynamicConstraints << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (cu::raw(positions), cu::raw(invMasses), thickness,
		cu::raw(vfContacts), vfContactsSize,
		cu::raw(eeContacts), eeContactsSize,
		cu::raw(m_dbDynamicCorrectionID.buffers[m_dbDynamicCorrectionID.selector]),
		cu::raw(m_dbDynamicCorrectionValues.buffers[m_dbDynamicCorrectionValues.selector]));

	cudaCheckError();


	CubWrap::SortByKey(m_dbDynamicCorrectionID, m_dbDynamicCorrectionValues, totalCorrections, tempStorage, tempStorageSize);
	CubWrap::RLE(m_dbDynamicCorrectionID.buffers[m_dbDynamicCorrectionID.selector], totalCorrections, m_dDynamicCorrectionRLEUniques, m_dDynamicCorrectionRLECounts, m_dynamicCorrectionsRunCount,
		tempStorage, tempStorageSize);

	CubWrap::ExclusiveSumInPlace(m_dDynamicCorrectionRLECounts, m_dynamicCorrectionsRunCount, tempStorage, tempStorageSize);
	

	numBlocks = (m_dynamicCorrectionsRunCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_ApplyDynamicCorrections << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_dynamicCorrectionsRunCount, cu::raw(m_dDynamicCorrectionRLEUniques), cu::raw(m_dDynamicCorrectionRLECounts),
		cu::raw(m_dbDynamicCorrectionValues.buffers[m_dbDynamicCorrectionValues.selector]), totalCorrections, cu::raw(m_accumulatedCorrectionValues), cu::raw(m_accumulatedCorrectionCounts));

	cudaCheckError();

}

void CudaPBD::CudaPBD::ProjectStaticConstraints(const thrust::device_vector<float3>& positions, const thrust::device_vector<float>& invMasses, const int iteration)
{
	const int stretchCount = (int)m_stretchConstraints.id1.size();
	const int bendCount = (int)m_bendConstraints.id1.size();

	int numBlocks = (stretchCount + bendCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_ProjectStaticConstraints << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (stretchCount, bendCount, iteration,
		cu::raw(positions), cu::raw(invMasses),
		cu::raw(m_stretchConstraints.id1), cu::raw(m_stretchConstraints.id2), cu::raw(m_stretchConstraints.id1Offset), cu::raw(m_stretchConstraints.id2Offset),
		m_stretchConstraints.k, cu::raw(m_stretchConstraints.l0),
		cu::raw(m_bendConstraints.id1), cu::raw(m_bendConstraints.id2), cu::raw(m_bendConstraints.id3), cu::raw(m_bendConstraints.id4),
		cu::raw(m_bendConstraints.id1Offset), cu::raw(m_bendConstraints.id2Offset), cu::raw(m_bendConstraints.id3Offset), cu::raw(m_bendConstraints.id4Offset),
		m_bendConstraints.k, cu::raw(m_bendConstraints.phi0), cu::raw(m_staticCorrectionValues));

	cudaCheckError();

	const int particleCount = (int)positions.size();
	numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_ApplyStaticCorrections << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> >(particleCount, cu::raw(m_staticCorrectionValues),
		cu::raw(m_staticCorrectionPrefixSums), cu::raw(m_accumulatedCorrectionValues), cu::raw(m_accumulatedCorrectionCounts));

	cudaCheckError();

}

void CudaPBD::CudaPBD::ProjectGroundConstraints(const thrust::device_vector<float3>& positions, const thrust::device_vector<float>& invMasses)
{
	const int particleCount = (int)positions.size();

	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_ApplyGroundCorrections << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), cu::raw(invMasses),
		cu::raw(m_groundConstraints.flag), cu::raw(m_groundConstraints.qc), cu::raw(m_groundConstraints.nc), cu::raw(m_accumulatedCorrectionValues), cu::raw(m_accumulatedCorrectionCounts));
}

void CudaPBD::CudaPBD::ProjectLRAConstraints(const thrust::device_vector<float3>& positions, const thrust::device_vector<float>& invMasses)
{
	int numBlocks = (m_particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_ApplyLRACorrections << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (m_particleCount, m_LRAConstraints.fixedVertCount,
		cu::raw(positions), cu::raw(invMasses), cu::raw(m_LRAConstraints.fixedPosID), cu::raw(m_LRAConstraints.l0), cu::raw(m_LRAConstraints.flag),
		cu::raw(m_accumulatedCorrectionValues), cu::raw(m_accumulatedCorrectionCounts));
}


void CudaPBD::CudaPBD::ApplyCorrections(thrust::device_vector<float3>& positions)
{
	const int particleCount = m_accumulatedCorrectionValues.size();
	const int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	FinalCorrectionStep << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(m_accumulatedCorrectionValues), cu::raw(m_accumulatedCorrectionCounts), cu::raw(positions));
	cudaCheckError();

}

void CudaPBD::CudaPBD::FinalUpdate(thrust::device_vector<float3>& positions, thrust::device_vector<float3>& prevPositions, thrust::device_vector<float3>& velocities,
	const thrust::device_vector<bool> &fixedVerts)
{
	const int particleCount = (int)positions.size();
	int numBlocks = (particleCount + CudaUtils::THREADS_PER_BLOCK - 1) / CudaUtils::THREADS_PER_BLOCK;

	_FinalUpdate << <numBlocks, CudaUtils::THREADS_PER_BLOCK >> > (particleCount, cu::raw(positions), cu::raw(prevPositions), cu::raw(velocities), 
		cu::raw(fixedVerts), m_parentParams.timestep);

	cudaCheckError();

}

__global__ void CudaPBD::_ProjectStaticConstraints(const int stretchCount, const int bendCount, const int iteration,
	const float3 * __restrict__ positions, const float * __restrict__ invMasses,
	const int *__restrict__ sid1, const int *__restrict__ sid2, const int *__restrict__ sid1o, const int *__restrict__ sid2o,
	const float sk, const float *__restrict__ sl0,
	const int *__restrict__ bid1, const int *__restrict__ bid2, const int *__restrict__ bid3, const int *__restrict__ bid4,
	const int *__restrict__ bid1o, const int *__restrict__ bid2o, const int *__restrict__ bid3o, const int *__restrict__ bid4o,
	const float bk, const float *__restrict__ bphi0, float3 *__restrict__ corVals)
{
	int id = CudaUtils::MyID();
	if (id >= stretchCount + bendCount)
		return;

	if (id < stretchCount)
	{
		ProjectStaticStretchConstraint(id, iteration, positions, invMasses, sid1, sid2, sid1o, sid2o, sk, sl0, corVals);
	}
	else
	{
		ProjectStaticBendConstraint(id - stretchCount, iteration, positions, invMasses, bid1, bid2, bid3, bid4, bid1o, bid2o, bid3o, bid4o, bk, bphi0, corVals);
	}
}

__device__ void CudaPBD::ProjectStaticStretchConstraint(const int id, const int iteration, const float3 *__restrict__ positions, const float *__restrict__ invMasses,
	const int *__restrict__ sid1, const int *__restrict__ sid2, const int *__restrict__ sid1o, const int *__restrict__ sid2o,
	const float sk, const float *__restrict__ sl0, float3 *__restrict__ corVals)
{
	const float kp = 1 - powf(1 - sk, 1.f / iteration);
	const float w1 = invMasses[sid1[id]];
	const float w2 = invMasses[sid2[id]];

	const float wSum = w1 + w2;

	if (wSum == 0)
	{
		corVals[sid1o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[sid2o[id]] = make_float3(0.f, 0.f, 0.f);

		printf("[%d] Stretch wSum = 0\n", id);
		return;
	}

	const float3 diff = positions[sid1[id]] - positions[sid2[id]];
	const float len = CudaUtils::len(diff);

	if (len <= EPS)
	{
		corVals[sid1o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[sid2o[id]] = make_float3(0.f, 0.f, 0.f);
		printf("[%d] Stretch len = 0\n", id);
		return;
	}

	const float3 n = diff / len;

	const float3 tempTerm = (len - sl0[id]) * n * kp / wSum;
	
	corVals[sid1o[id]] = -w1 * tempTerm;
	corVals[sid2o[id]] = w2  * tempTerm;
}

__device__ void CudaPBD::ProjectStaticBendConstraint(const int id, const int iteration, const float3 *__restrict__ positions, const float *__restrict__ invMasses,
	const int *__restrict__ bid1, const int *__restrict__ bid2, const int *__restrict__ bid3, const int *__restrict__ bid4,
	const int *__restrict__ bid1o, const int *__restrict__ bid2o, const int *__restrict__ bid3o, const int *__restrict__ bid4o,
	const float bk, const float *__restrict__ bphi0, float3 *__restrict__ corVals)
{
	const float kp = 1.f - powf(1.f - bk, 1.f / iteration);
	const float w1 = invMasses[bid1[id]];
	const float w2 = invMasses[bid2[id]];
	const float w3 = invMasses[bid3[id]];
	const float w4 = invMasses[bid4[id]];

	const float3 p1 = positions[bid1[id]];
	const float3 p2 = positions[bid2[id]];
	const float3 p3 = positions[bid3[id]];
	const float3 p4 = positions[bid4[id]];

	const float3 p21 = p2 - p1;

	const float3 c23 = CudaUtils::cross(p2, p3);
	const float3 c24 = CudaUtils::cross(p2, p4);

	const float l23 = CudaUtils::len(c23);
	const float l24 = CudaUtils::len(c24);

	if (l23 <= EPS || l24 <= EPS)
	{
		corVals[bid1o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[bid2o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[bid3o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[bid4o[id]] = make_float3(0.f, 0.f, 0.f);

		printf("[%d] Bend l23: %f, l24: %f\n", id, l23, l24);
		return;
	}

	const float3 n1 = c23 / l23;
	const float3 n2 = c24 / l24;

	const float d = CudaUtils::clamp(CudaUtils::dot(n1, n2), -1.f, 1.f);

	const float3 q3 = (CudaUtils::cross(p2, n2) + (CudaUtils::cross(n1, p2) * d)) / l23;

	//if (CudaUtils::isZero(q3))
	//{
	//	printf("[%d] q3 zero p2: (%g, %g, %g), n2: (%g, %g, %g), n1: (%g, %g, %g)\n", id, p2.x, p2.y, p2.y, n2.x, n2.y, n2.z, n1.x, n1.y, n1.z);

	//}
	const float3 q4 = (CudaUtils::cross(p2, n1) + (CudaUtils::cross(n2, p2) * d)) / l24;

	/*if (CudaUtils::isZero(q4))
	{
		printf("[%d] q4 zero p2: (%g, %g, %g), n2: (%g, %g, %g), n1: (%g, %g, %g)\n", id, p2.x, p2.y, p2.y, n2.x, n2.y, n2.z, n1.x, n1.y, n1.z);

	}*/
	const float3 q2 = -((CudaUtils::cross(p3, n2) + (CudaUtils::cross(n1, p3) * d)) / l23) - ((CudaUtils::cross(p4, n1) + (CudaUtils::cross(n2, p4) * d)) / l24);

	/*if (CudaUtils::isZero(q2))
	{
		printf("[%d] q2 zero p3: (%g, %g, %g), n2: (%g, %g, %g), n1: (%g, %g, %g)\n", id, p3.x, p3.y, p3.y, n2.x, n2.y, n2.z, n1.x, n1.y, n1.z);

	}*/
	const float3 q1 = -q2 - q3 - q4;

	const float term = sqrtf(1.f - d * d) * (acosf(d) - bphi0[id]) * kp;

	const float l1 = CudaUtils::len(q1);
	const float l2 = CudaUtils::len(q2);
	const float l3 = CudaUtils::len(q3);
	const float l4 = CudaUtils::len(q4);

	const float scalingFactor = w1 * l1 * l1 + w2 * l2 * l2 + w3 * l3 * l3 + w4 * l4 * l4;

	if (scalingFactor <= EPS)
	{
		corVals[bid1o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[bid2o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[bid3o[id]] = make_float3(0.f, 0.f, 0.f);
		corVals[bid4o[id]] = make_float3(0.f, 0.f, 0.f);

		printf("[%d] Bend scalingFactor = 0, w1: %g, l1: %g, w2: %g, l2: %g, w3: %g, l3: %g, w4: %g, l4: %g, d = %g\n", id, w1, l1, w2, l2, w3, l3, w4, l4, d);
		return;
	}

	const float ts = term / scalingFactor;

	corVals[bid1o[id]] = -(w1 * ts) * q1;
	corVals[bid2o[id]] = -(w2 * ts) * q2;
	corVals[bid3o[id]] = -(w3 * ts) * q3;
	corVals[bid4o[id]] = -(w4 * ts) * q4;
}


__global__ void CudaPBD::_ProjectDynamicConstraints(const float3 * __restrict__ positions, const float * __restrict__ invMasses,
	const float thickness,
	const Physics::PrimitiveContact * __restrict__ vfs, const uint64_t vfSize,
	const Physics::PrimitiveContact * __restrict__ ees, const uint64_t eeSize,
	uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals)
{
	const int id = CudaUtils::MyID();
	if (id >= vfSize + eeSize)
		return;

	if (id < vfSize)
	{
		ProjectVFConstraint(id, id * 4, positions, invMasses, thickness, vfs, rawCorIDs, rawCorVals);
	}
	else
	{
		ProjectEEConstraint(id - vfSize, id * 4, positions, invMasses, thickness, ees, rawCorIDs, rawCorVals);
	}
}

__device__ void CudaPBD::ProjectVFConstraint(const int id, const int myStart,
	const float3 * __restrict__ positions,
	const float * __restrict__ invMasses, const float thickness,
	const Physics::PrimitiveContact * __restrict__ vfs,
	uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals)
{
	
	const Physics::PrimitiveContact myContact = vfs[id];

	const float b0 = CudaUtils::clamp(-myContact.w2, 0.f, 1.f);
	const float b1 = CudaUtils::clamp(-myContact.w3, 0.f, 1.f);
	const float b2 = CudaUtils::clamp(-myContact.w4, 0.f, 1.f);


	const float3 p = positions[myContact.v1];
	const float3 p0 = positions[myContact.v2];
	const float3 p1 = positions[myContact.v3];
	const float3 p2 = positions[myContact.v4];

	const float w = invMasses[myContact.v1];
	const float w0 = invMasses[myContact.v2];
	const float w1 = invMasses[myContact.v3];
	const float w2 = invMasses[myContact.v4];

	float3 q = p0 * b0 + p1 * b1 + p2 * b2;
	float3 n = p - q;

	float len = CudaUtils::len(n);

	n /= len;

	float c = len - thickness;

	float3 grad = n;
	float3 grad0 = -n * b0;
	float3 grad1 = -n * b1;
	float3 grad2 = -n * b2;

	rawCorIDs[myStart] = myContact.v1;
	rawCorIDs[myStart + 1] = myContact.v2;
	rawCorIDs[myStart + 2] = myContact.v3;
	rawCorIDs[myStart + 3] = myContact.v4;

	float s = w + w0 * b0 * b0 + w1 * b1 * b1 + w2 * b2 * b2;

	if (s == 0)
	{
		return;
	}

	s = c / s;

	s = c < 0 ? s : 0;

	if (s == 0)
	{
		return;
	}

	rawCorVals[myStart] = -s * w * grad;
	rawCorVals[myStart + 1] = -s * w0 * grad0;
	rawCorVals[myStart + 2] = -s * w1 * grad1;
	rawCorVals[myStart + 3] = -s * w2 * grad2;

}

__device__ void CudaPBD::ProjectEEConstraint(const int id, const int myStart,
	const float3 * __restrict__ positions,
	const float * __restrict__ invMasses, const float thickness,
	const Physics::PrimitiveContact * __restrict__ ees,
	uint32_t * __restrict__ rawCorIDs, float3 * __restrict__ rawCorVals)
{
	const Physics::PrimitiveContact myContact = ees[id];


	rawCorIDs[myStart] = myContact.v1;
	rawCorIDs[myStart + 1] = myContact.v2;
	rawCorIDs[myStart + 2] = myContact.v3;
	rawCorIDs[myStart + 3] = myContact.v4;

	const float3 p0 = positions[myContact.v1];
	const float3 p1 = positions[myContact.v2];
	const float3 p2 = positions[myContact.v3];
	const float3 p3 = positions[myContact.v4];

	const float w0 = invMasses[myContact.v1];
	const float w1 = invMasses[myContact.v2];
	const float w2 = invMasses[myContact.v3];
	const float w3 = invMasses[myContact.v4];


	const float b0 = myContact.w1;
	const float b1 = myContact.w2;
	const float b2 = -myContact.w3;
	const float b3 = -myContact.w4;

	float3 q0 = p0 * b0 + p1 * b1;
	float3 q1 = p2 * b2 + p3 * b3;

	float3 n = q0 - q1;

	float len = CudaUtils::len(n);
	n /= len;

	float c = len - thickness;

	float3 grad0 = n * b0;
	float3 grad1 = n * b1;
	float3 grad2 = -n * b2;
	float3 grad3 = -n * b3;

	float s = w0 * b0 * b0 + w1 * b1 * b1 + w2 * b2 * b2 + w3 * b3 * b3;

	if (s == 0)
		return;

	s = c / s;

	s = c < 0 ? s : 0;

	if (s == 0)
	{
		return;
	}

	rawCorVals[myStart] = -s * w0 * grad0;
	rawCorVals[myStart + 1] = -s * w1 * grad1;
	rawCorVals[myStart + 2] = -s * w2 * grad2;
	rawCorVals[myStart + 3] = -s * w3 * grad3;
}

__global__ void CudaPBD::_ApplyStaticCorrections(const int particleCount, const float3 * __restrict__ rawValues,
	const int * __restrict__ prefixSums,
	float3 * __restrict__ accumulatedValues, int * __restrict__ accumulatedCounts)
{
	const int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	const int myStart = prefixSums[id];
	const int myEnd = prefixSums[id + 1];
	//const int count = myEnd - myStart;


	float3 acc = make_float3(0.f, 0.f, 0.f);

	for (int i = myStart; i < myEnd; ++i)
	{
		if (CudaUtils::isNan(rawValues[i]))
		{
			printf("[%d] rawValues[%d] is NaN\n", id, i);
		}
		acc = acc + rawValues[i];
	}

	//printf("[%d] acc: (%f, %f, %f)\n", id, acc.x, acc.y, acc.z);

	//positions[id] += (acc / count);
	accumulatedValues[id] += acc;
	accumulatedCounts[id] += (myEnd - myStart);
}

__global__ void CudaPBD::_ApplyDynamicCorrections(const int runCount, const uint32_t *__restrict__ RLEUniques,
	const int *__restrict__ RLEPrefixSums, const float3 *__restrict__ rawCorVals,
	const uint64_t rawCorValsSize, float3 *__restrict__ accumulatedCors, int * __restrict__ accumulatedCounts)
{
	int id = CudaUtils::MyID();
	if (id >= runCount)
		return;

	int myAccImpulseID = RLEUniques[id];

	int myStart = RLEPrefixSums[id];
	int myEnd = id == runCount - 1 ? rawCorValsSize : RLEPrefixSums[id + 1];

	float3 acc = make_float3(0.f, 0.f, 0.f);

	for (int i = myStart; i < myEnd; ++i)
	{
		acc += rawCorVals[i];
	}

	accumulatedCors[myAccImpulseID] += acc;
	accumulatedCounts[myAccImpulseID] += (myEnd - myStart);
}

__global__ void CudaPBD::_CreateGroundConstraints(const int particleCount,
	const float3 * __restrict__ positions, const float3 * __restrict__ prevPositions,
	const float * __restrict__ invMasses, const float groundHeight, const float thickness,
	float3 * __restrict__ qcs, float3 * __restrict__ ncs, bool * __restrict__ flags)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	float3 pos = positions[id];
	float3 prevPos = prevPositions[id];


	if (pos.y <= groundHeight + thickness)
	{
		float3 qc;
		float3 nc = make_float3(0.f, 1.f, 0.f);
		float3 p0 = make_float3(1.f, groundHeight + thickness, 1.f);

		float3 l = pos - prevPos;

		float d = CudaUtils::dot(p0 - pos, nc) / CudaUtils::dot(l, nc);

		qc = d * l + pos;

		flags[id] = true;
		qcs[id] = qc;
		ncs[id] = nc;

	}
}

__global__ void CudaPBD::_CreateLRAConstraints(const int particleCount, const int fixedCount,
	const float3 *__restrict__ positions, const int *__restrict__ fixedPosIDs, const float *__restrict__ l0s, bool *__restrict__ flags)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount * fixedCount)
		return;


	float3 myPos = positions[id / fixedCount];
	float3 myFixed = positions[fixedPosIDs[id % fixedCount]];

	if (CudaUtils::distance(myPos, myFixed) > l0s[id])
	{
		flags[id] = true;
	}
}

__global__ void CudaPBD::_ApplyGroundCorrections(const int particleCount, const float3 *__restrict__ positions,
	const float *__restrict__ invMasses, const bool *__restrict__ flags, const float3 *__restrict__ qcs, float3 *__restrict__ ncs,
	float3 *__restrict__ accumulatedCors, int *__restrict__ accumulatedCounts)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	if (flags[id])
	{
		float3 pos = positions[id];
		float3 qc = qcs[id];
		float3 nc = ncs[id];

		float w = invMasses[id];

		float cVal = CudaUtils::dot(pos - qc, nc);

		if (cVal < 0)
		{
			float deriv = nc.x + nc.y + nc.z;

			
			accumulatedCors[id] += -(cVal / (w * deriv * deriv)) * w * nc;
			accumulatedCounts[id] += 1;
		}
	}
}

__global__ void CudaPBD::_ApplyLRACorrections(const int particleCount, const int fixedCount,
	const float3 *__restrict__ positions, const float *__restrict__ invMasses,
	const int *__restrict__ fixedIDs, const float *__restrict__ l0s, const bool *__restrict__ flags,
	float3 *__restrict__ accumulatedCors, int *__restrict__ accumulatedCounts)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	if (l0s[id] == 0.f)
		return;


	float3 acc = make_float3(0.f, 0.f, 0.f);
	int cnt = 0;

	float3 myPos = positions[id];

	for (int i = 0; i < fixedCount; ++i)
	{
		if (flags[id * fixedCount + i])
		{
			float3 fixedPos = positions[fixedIDs[i]];

			

			float3 diff = myPos - fixedPos;
			float len = CudaUtils::len(diff);

			float3 n = diff / len;

			acc += -(len - l0s[id + i]) * n;
			cnt++;
		}
	}

	accumulatedCors[id] += acc;
	accumulatedCounts[id] += cnt;
}

__global__ void CudaPBD::FinalCorrectionStep(const int particleCount,
	const float3 *__restrict__ accumulatedCors, const int *__restrict__ accumulatedCounts, float3 *__restrict__ positions)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	positions[id] += (accumulatedCors[id] / accumulatedCounts[id]);
}



__global__ void CudaPBD::_FinalUpdate(const int particleCount, float3 *__restrict__ positions, float3 *__restrict__ prevPositions,
	float3 *__restrict__ velocities, const bool * __restrict__ fixedVerts, const float timestep)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	if (fixedVerts[id])
	{
		velocities[id] = make_float3(0.f, 0.f, 0.f);		
		return;
	}
		

	velocities[id] = (positions[id] - prevPositions[id]) / timestep;
	prevPositions[id] = positions[id];
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

__global__ void CudaPBD::ApplyExternalForces(const int particleCount, float3 *__restrict__ vels, const float3 gravity, const float timestep)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	vels[id] = vels[id] + timestep * gravity;
}

__global__ void CudaPBD::ExplicitEuler(const int particleCount, const float3 *__restrict__ prevPositions, const float3 *__restrict__ velocities, float3 *__restrict__ positions, const float timestep)
{
	int id = CudaUtils::MyID();
	if (id >= particleCount)
		return;

	positions[id] = prevPositions[id] + timestep * velocities[id];
}





//void CudaPBD::CudaPBD::CreateStaticBendConstraint2(const int3 tri1, const int3 tri2, const std::vector<Rendering::VertexFormat>& verts,
//	thrust::host_vector<int>& bid1, thrust::host_vector<int>& bid2, thrust::host_vector<int>& bid3, thrust::host_vector<int>& bid4,
//	thrust::host_vector<float>& bphi0)
//{
//	int p1 = -1, p2 = -1, p3 = -1, p4 = -1;
//
//	int t1[3], t2[3];
//	t1[0] = tri1.x;
//	t1[1] = tri1.y;
//	t1[2] = tri1.z;
//
//	t2[0] = tri2.x;
//	t2[1] = tri2.y;
//	t2[2] = tri2.z;
//
//
//
//	for (int i = 0; i < 3; ++i)
//	{
//		for (int j = 0; j < 3; ++j)
//		{
//			if (t1[i] == t2[j])
//			{
//				p3 = t1[i];
//				break;
//			}
//		}
//	}
//
//	for (int i = 0; i < 3; ++i)
//	{
//		for (int j = 0; j < 3; ++j)
//		{
//			if (t1[i] == t2[j] && t1[i] != p1)
//			{
//				p4 = t1[i];
//				break;
//			}
//		}
//	}
//
//	for (int i = 0; i < 3; ++i)
//	{
//		if (t1[i] != p3 && t1[i] != p4)
//		{
//			p1 = t1[i];
//			break;
//		}
//	}
//
//	for (int i = 0; i < 3; ++i)
//	{
//		if (t2[i] != p3 && t2[i] != p4)
//		{
//			p2 = t2[i];
//			break;
//		}
//	}
//
//
//
//	bid1.push_back(p1);
//	bid2.push_back(p2);
//	bid3.push_back(p3);
//	bid4.push_back(p4);
//	bphi0.push_back(PI);
//
//}




//void CudaPBD::CudaPBD::ProjectConstraints(thrust::device_vector<float3> &positions, const thrust::device_vector<float> &invMasses,
//	const int iteration,
//	const thrust::device_vector<Physics::PrimitiveContact>& vfContacts,
//	const uint64_t & vfContactsSize,
//	const thrust::device_vector<Physics::PrimitiveContact>& eeContacts,
//	const uint64_t & eeContactsSize,
//	void *& tempStorage, uint64_t & tempStorageSize)
//{
//	
//
//
//	ProjectStaticConstraints(positions, invMasses, iteration);
//	cudaCheckError();
//
//
//
//	ProjectDynamicConstraints(positions, invMasses, m_parentParams.thickness, vfContacts, vfContactsSize, eeContacts, eeContactsSize, tempStorage, tempStorageSize);
//	cudaCheckError();
//
//	ApplyCorrections(positions);	
//}









//__device__ void CudaPBD::ProjectStaticBendConstraint2(const int id, const int iteration, const float3 *__restrict__ positions,
//	const float *__restrict__ invMasses,
//	const int *__restrict__ bid1, const int *__restrict__ bid2, const int *__restrict__ bid3, const int *__restrict__ bid4,
//	const int *__restrict__ bid1o, const int *__restrict__ bid2o, const int *__restrict__ bid3o, const int *__restrict__ bid4o,
//	const float bk, const float *__restrict__ bphi0, float3 *__restrict__ corVals)
//{
//	const float kp = 1.f - powf(1.f - bk, 1.f / iteration);
//
//	const float w0 = invMasses[bid1[id]];
//	const float w1 = invMasses[bid2[id]];
//	const float w2 = invMasses[bid3[id]];
//	const float w3 = invMasses[bid4[id]];
//
//	const float3 p0 = positions[bid1[id]];
//	const float3 p1 = positions[bid2[id]];
//	const float3 p2 = positions[bid3[id]];
//	const float3 p3 = positions[bid4[id]];
//
//
//	if (w0 == 0.0 && w1 == 0.0)
//	{
//		corVals[bid1o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid2o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid3o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid4o[id]] = make_float3(0.f, 0.f, 0.f);
//
//		return;
//	}
//
//	float3 e = p3 - p2;
//
//	float elen = CudaUtils::len(e);
//
//	if (elen < EPS)
//	{
//		corVals[bid1o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid2o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid3o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid4o[id]] = make_float3(0.f, 0.f, 0.f);
//
//		return;
//	}
//
//	float invElen = 1.f / elen;
//
//	float3 n1 = CudaUtils::cross(p2 - p0, p3 - p0);
//	float3 n2 = CudaUtils::cross(p3 - p1, p2 - p0);
//
//	float nl1 = CudaUtils::len(n1);
//	float nl2 = CudaUtils::len(n2);
//
//	n1 /= (nl1 * nl1);
//	n2 /= (nl2 * nl2);
//
//	float3 d0 = elen * n1;
//	float3 d1 = elen * n2;
//	float3 d2 = CudaUtils::dot(p0 - p3, e) * invElen * n1 + CudaUtils::dot(p1 - p3, e) * invElen * n2;
//	float3 d3 = CudaUtils::dot(p2 - p0, e) * invElen * n1 + CudaUtils::dot(p2 - p1, e) * invElen * n2;
//
//
//	
//	float d0len = CudaUtils::len(d0);
//	float d1len = CudaUtils::len(d1);
//	float d2len = CudaUtils::len(d2);
//	float d3len = CudaUtils::len(d3);
//
//	n1 = CudaUtils::normalize(n1);
//	n2 = CudaUtils::normalize(n2);
//
//	float dot = CudaUtils::clamp(CudaUtils::dot(n1, n2), -1.f, 1.f);
//
//	float phi = acosf(dot);
//
//	float lambda = w0 * d0len * d0len + w1 * d1len * d1len + w2 * d2len * d2len + w3 * d3len * d3len;
//
//	if (lambda == 0)
//	{
//		corVals[bid1o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid2o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid3o[id]] = make_float3(0.f, 0.f, 0.f);
//		corVals[bid4o[id]] = make_float3(0.f, 0.f, 0.f);
//
//		return;
//	}
//
//
//	lambda = (phi - bphi0[id]) / lambda * kp;
//
//	if (CudaUtils::dot(CudaUtils::cross(n1, n2), e) > 0)
//		lambda = -lambda;
//
//	corVals[bid1o[id]] = -(w0 * lambda) * d0;
//	corVals[bid2o[id]] = -(w1 * lambda) * d1;
//	corVals[bid3o[id]] = -(w2 * lambda) * d2;
//	corVals[bid4o[id]] = -(w3 * lambda) * d3;
//
//}
