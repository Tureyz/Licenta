#pragma once
#include "ICollisionMethod.h"
#include "DataStructures\BVHTree.h"

namespace Collision
{
	class BVH :
		public ICollisionMethod
	{
	public:
		BVH(std::vector<Rendering::IPhysicsObject *> *allObjects);

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		void CreateTree(DataStructures::BVHTree **node, Rendering::IPhysicsObject ** objects, size_t numObjects);

		size_t SplitObjects(Collision::DataStructures::BVHTree *node);

	protected:
		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		void DrawRecursive(DataStructures::BVHTree *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		bool ChildrenSelectionRule(DataStructures::BVHTree *left, DataStructures::BVHTree *right);

		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> QueryBVHPairs(DataStructures::BVHTree *first, DataStructures::BVHTree *second);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		// TODO see if performance is better if tree is kept as an array (left = 2 * i, right = 2 * i + 1)
		DataStructures::BVHTree *m_root;
	};
}
