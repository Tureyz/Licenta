#pragma once
#include "ICollisionMethod.h"
#include "DataStructures\BVHTree.h"

namespace Collision
{
	class BVH :	public ICollisionMethod
	{
	public:
		BVH(std::vector<Rendering::IPhysicsObject *> *allObjects);

		virtual void DrawDebug(const glm::mat4& viewProjection) override;

		void CreateTree(DataStructures::BVHTree<Rendering::IPhysicsObject *> **node, Rendering::IPhysicsObject ** objects, size_t numObjects);

		size_t SplitObjects(Collision::DataStructures::BVHTree<Rendering::IPhysicsObject *> *node);

	protected:
		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		void DrawRecursive(DataStructures::BVHTree<Rendering::IPhysicsObject *> *node, const glm::mat4& viewProjection);

		bool ChildrenSelectionRule(DataStructures::BVHTree<Rendering::IPhysicsObject *> *left, DataStructures::BVHTree<Rendering::IPhysicsObject *> *right);

		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> QueryBVHPairs(DataStructures::BVHTree<Rendering::IPhysicsObject *> *first, DataStructures::BVHTree<Rendering::IPhysicsObject *> *second);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		// TODO see if performance is better if tree is kept as an array (left = 2 * i, right = 2 * i + 1)
		DataStructures::BVHTree<Rendering::IPhysicsObject *> *m_root;
	};
}
