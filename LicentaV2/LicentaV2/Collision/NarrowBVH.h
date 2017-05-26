#pragma once
#include "INarrowPhaseMethod.h"
#include "DataStructures\BVHTree.h"

namespace Collision
{
	class NarrowBVH : public INarrowPhaseMethod
	{
	public:
		NarrowBVH(std::vector<Collision::DataStructures::CollisionTriangle *> *triangles);

		~NarrowBVH();
		virtual std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> TestCollision(NarrowBVH *other);

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;
		void CreateTree(DataStructures::BVHTree<DataStructures::CollisionTriangle *> **node, DataStructures::CollisionTriangle **objects, size_t numObjects);

		virtual void ObjectMoved(Collision::DataStructures::CollisionTriangle *object) override;

		virtual void ObjectAdded(Collision::DataStructures::CollisionTriangle *object) override;

		virtual void ObjectRemoved(Collision::DataStructures::CollisionTriangle *object) override;

		size_t SplitObjects(Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node);

	private:

		void DrawRecursive(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		bool ChildrenSelectionRule(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *left, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *right);

		std::unordered_set<std::pair<DataStructures::CollisionTriangle *, DataStructures::CollisionTriangle *>>
			QueryBVHPairs(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *first, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *second);

		std::unordered_set<std::pair<DataStructures::CollisionTriangle *, DataStructures::CollisionTriangle *>>
			QueryBVHPairsLoop(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *first, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *second);

		void QueryBVHPairsRecursive(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *first, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *second, std::unordered_set<std::pair<DataStructures::CollisionTriangle *, DataStructures::CollisionTriangle *>> &result, int indent = 0);

		DataStructures::BVHTree<DataStructures::CollisionTriangle *> *m_root;


		void PrintIDsRecursive(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node);

	};
}
