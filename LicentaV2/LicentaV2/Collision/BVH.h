#pragma once
#include "ICollisionMethod.h"
#include "DataStructures\BVHTree.h"

namespace Collision
{
	class BVH :	public ICollisionMethod
	{
	public:
		BVH(std::vector<Rendering::SceneObject *> *allObjects);

		virtual void DrawDebug(const glm::mat4& viewProjection) override;

		void CreateTree(DataStructures::BVHTree<Rendering::SceneObject *> **node, Rendering::SceneObject ** objects, size_t numObjects);

		size_t SplitObjects(Collision::DataStructures::BVHTree<Rendering::SceneObject *> *node);

	protected:
		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		void DrawRecursive(DataStructures::BVHTree<Rendering::SceneObject *> *node, const glm::mat4& viewProjection);

		bool ChildrenSelectionRule(DataStructures::BVHTree<Rendering::SceneObject *> *left, DataStructures::BVHTree<Rendering::SceneObject *> *right);

		std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> QueryBVHPairs(DataStructures::BVHTree<Rendering::SceneObject *> *first, DataStructures::BVHTree<Rendering::SceneObject *> *second);

		virtual void ObjectMoved(Rendering::SceneObject *object) override;

		virtual void ObjectAdded(Rendering::SceneObject *object) override;

		virtual void ObjectRemoved(Rendering::SceneObject *object) override;

		// TODO see if performance is better if tree is kept as an array (left = 2 * i, right = 2 * i + 1)
		DataStructures::BVHTree<Rendering::SceneObject *> *m_root;

		void _DeleteTree();
	};
}
