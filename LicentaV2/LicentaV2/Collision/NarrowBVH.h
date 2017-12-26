#pragma once
#include "INarrowPhaseMethod.h"
#include "DataStructures\BVHTree.h"

namespace Collision
{
	class NarrowBVH : public INarrowPhaseMethod
	{
	public:
		NarrowBVH(std::vector<Physics::CollisionTriangle *> *triangles);

		~NarrowBVH();
		virtual std::unordered_set<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>> TestCollision(NarrowBVH *other);

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;
		void CreateTree(DataStructures::BVHTree<Physics::CollisionTriangle *> **node, Physics::CollisionTriangle **objects, size_t numObjects);

		virtual void ObjectMoved(Physics::CollisionTriangle *object) override;

		virtual void ObjectAdded(Physics::CollisionTriangle *object) override;

		virtual void ObjectRemoved(Physics::CollisionTriangle *object) override;

		size_t SplitObjects(Collision::DataStructures::BVHTree<Physics::CollisionTriangle *> *node);

	private:

		void DrawRecursive(DataStructures::BVHTree<Physics::CollisionTriangle *> *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		bool ChildrenSelectionRule(DataStructures::BVHTree<Physics::CollisionTriangle *> *left, DataStructures::BVHTree<Physics::CollisionTriangle *> *right);

		std::unordered_set<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>>
			QueryBVHPairs(DataStructures::BVHTree<Physics::CollisionTriangle *> *first, DataStructures::BVHTree<Physics::CollisionTriangle *> *second);

		std::unordered_set<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>>
			QueryBVHPairsLoop(DataStructures::BVHTree<Physics::CollisionTriangle *> *first, DataStructures::BVHTree<Physics::CollisionTriangle *> *second);

		void QueryBVHPairsRecursive(DataStructures::BVHTree<Physics::CollisionTriangle *> *first, DataStructures::BVHTree<Physics::CollisionTriangle *> *second, std::unordered_set<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>> &result, int indent = 0);

		DataStructures::BVHTree<Physics::CollisionTriangle *> *m_root;


		void PrintIDsRecursive(DataStructures::BVHTree<Physics::CollisionTriangle *> *node);

		void UpdateBoxes(DataStructures::BVHTree<Physics::CollisionTriangle *> *node);

	};
}
