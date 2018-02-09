#include "RigidImpactZone.h"



#include "../Core/Utils.hpp"

void Physics::RigidImpactZone::SolveRigidImpactZones(Collision::NarrowSpatialHashing *narrowMethod, const std::vector<Physics::ClothNode *> &points, const std::vector<Physics::ClothTriangle *> &triangles, const std::vector<Physics::Edge *> &edges)
{

	for (size_t i = 0; i < points.size(); ++i)
	{
		points[i]->m_dsNode = new Physics::DisjointSet();
	}


	int cnt = 0;

	static size_t maxIterations = 10;

	for (size_t i = 0; i < maxIterations; ++i)
	{
		auto selfCols = narrowMethod->GetCollisions(points, triangles, edges, true);

		if (selfCols.first.empty() && selfCols.second.empty())
		{
			break;
		}

		for (auto ptContact : selfCols.first)
		{
			MergeZones(ptContact.m_point, ptContact.m_triangle->m_nodes[0], ptContact.m_triangle->m_nodes[1], ptContact.m_triangle->m_nodes[2]);
		}

		for (auto eeContact : selfCols.second)
		{
			MergeZones(eeContact.m_e1->m_v1, eeContact.m_e1->m_v2, eeContact.m_e2->m_v1, eeContact.m_e2->m_v2);
		}

		//std::cout << "Iteration " << cnt << ". Found " << selfCols.first.size() << " pt and " << +selfCols.second.size() << " ee collisions." << std::endl;
		cnt++;

		ApplyRigidVels(points);

	}
}

void Physics::RigidImpactZone::MergeZones(Physics::ClothNode *p1, Physics::ClothNode *p2, Physics::ClothNode *p3, Physics::ClothNode *p4)
{
	//convention: merge all into p1's zone


	Physics::DisjointSet::Union(p1->m_dsNode, p2->m_dsNode);
	Physics::DisjointSet::Union(p1->m_dsNode, p3->m_dsNode);
	Physics::DisjointSet::Union(p1->m_dsNode, p4->m_dsNode);


	// 	MergeTwoZones(impactZones, p1, p2);
	// 	MergeTwoZones(impactZones, p1, p3);
	// 	MergeTwoZones(impactZones, p1, p4);

}

void Physics::RigidImpactZone::MergeTwoZones(Physics::ClothNode *p1, Physics::ClothNode *p2)
{
	// 	if (impactZones.empty())
	// 		return;
	// 	if (p1->m_impactZoneID != p2->m_impactZoneID)
	// 	{
	// 		auto zone = impactZones[p2->m_impactZoneID];
	// 		impactZones[p1->m_impactZoneID].insert(zone.begin(), zone.end());
	// 
	// 		impactZones.erase(p2->m_impactZoneID);
	// 
	// 
	// 		for (auto mergedNode : zone)
	// 		{
	// 			mergedNode->m_impactZoneID = p1->m_impactZoneID;
	// 		}
	// 
	// 
	// 	}	
}

void Physics::RigidImpactZone::ApplyRigidVels(const std::vector<Physics::ClothNode *> &points)
{

	std::unordered_map<DisjointSet *, std::unordered_set<ClothNode *>> impactZones;

	for (auto point : points)
	{
		if (point->m_dsNode->m_rank > 0 && point->m_dsNode->m_parent == point->m_dsNode)
		{
			impactZones[point->m_dsNode].insert(point);
		}
	}

	for (auto point : points)
	{
		auto pRoot = DisjointSet::Find(point->m_dsNode);

		if (impactZones.count(pRoot))
		{
			impactZones[pRoot].insert(point);
		}
	}


	for (auto kvPair : impactZones)
	{
		ApplyRigidVel(kvPair.second);
	}
}

void Physics::RigidImpactZone::ApplyRigidVel(std::unordered_set<Physics::ClothNode *> impactZone)
{

	glm::vec3 xcm, vcm;

	float massSum = 0;

	for (Physics::ClothNode *node : impactZone)
	{
		massSum += node->m_mass;
		xcm += node->m_mass * node->m_pos;
		vcm += node->m_mass * node->m_averageVel;
	}


	if (massSum == 0)
	{
		std::cout << "massSum 0" << std::endl;
	}


	xcm /= massSum;
	vcm /= massSum;

	glm::vec3 L;

	for (Physics::ClothNode *node : impactZone)
	{
		L += node->m_mass * glm::cross(node->m_pos - xcm, node->m_averageVel - vcm);
	}

	glm::mat3 I(0);

	for (Physics::ClothNode *node : impactZone)
	{
		glm::vec3 ri = node->m_pos - xcm;
		glm::mat3 rim(
			glm::vec3(0, -ri.z, ri.y),
			glm::vec3(ri.z, 0, -ri.x),
			glm::vec3(-ri.y, ri.x, 0));

		I += rim * glm::transpose(rim) * node->m_mass;
	}

	glm::vec3 omega = glm::inverse(I) * L;

	float omegaLen = glm::length(omega);

	if (omegaLen == 0)
	{
		std::cout << "omegalen 0" << std::endl;
	}
	glm::vec3 omegaNorm = omega / omegaLen;

	float omegadt = omegaLen * Core::TIME_STEP;

	for (Physics::ClothNode *node : impactZone)
	{
		glm::vec3 xf = glm::dot((node->m_pos - xcm), omegaNorm)  * omegaNorm;
		glm::vec3 xr = node->m_pos - xcm - xf;


		glm::vec3 newPos = xcm + Core::TIME_STEP * vcm + xf + std::cos(omegadt) * xr + std::sin(omegadt) * glm::cross(omegaNorm, xr);

		node->m_averageVel = (newPos - node->m_pos) / Core::TIME_STEP;
	}

}
