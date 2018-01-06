#pragma once
#include "../Collision/NarrowSpatialHashing.h"
#include <unordered_map>
#include <unordered_set>

namespace Physics
{
	class RigidImpactZone
	{
	public:

		static void SolveRigidImpactZones(Collision::NarrowSpatialHashing *narrowMethod, const std::vector<Physics::ClothNode *> &points, const std::vector<Physics::ClothTriangle *> &triangles, const std::vector<Physics::Edge *> &edges);

	private:
		static void MergeZones(Physics::ClothNode *p1, Physics::ClothNode *p2, Physics::ClothNode *p3, Physics::ClothNode *p4);
		static void MergeTwoZones(Physics::ClothNode *p1, Physics::ClothNode *p2);
		static void ApplyRigidVels(const std::vector<Physics::ClothNode *> &points);
		static void ApplyRigidVel(std::unordered_set<Physics::ClothNode *> impactZone);
	};
}
