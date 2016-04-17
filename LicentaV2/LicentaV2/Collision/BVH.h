#pragma once
#include "ICollisionMethod.h"
#include "DataStructures\BVHTree.h"
#include <unordered_map>

namespace Collision
{
	class BVH :
		public ICollisionMethod
	{
	public:
		BVH(std::vector<IPhysicsObject *> *allObjects);

		virtual std::vector<IPhysicsObject *> TestCollision(IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		void MakeTopDownTree(DataStructures::BVHTree **node, IPhysicsObject ** objects, size_t numObjects);

		size_t MakeSubsets(Collision::DataStructures::BVHTree *node);
	private:

		void DrawRecursive(DataStructures::BVHTree *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		bool DescendDirection(DataStructures::BVHTree *left, DataStructures::BVHTree *right);

		std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> QueryBVHPairs(DataStructures::BVHTree *first, DataStructures::BVHTree *second);
		std::vector<IPhysicsObject *> QueryBVH(DataStructures::BVHTree *queriedLeaf);

		// TODO see if performance is better if tree is kept as an array (left = 2 * i, right = 2 * i + 1)
		//std::unordered_map<int, DataStructures::BVHTree *> m_leaves;
		DataStructures::BVHTree *m_root;
	};
}
