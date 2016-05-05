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

		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		void MakeTopDownTree(DataStructures::BVHTree **node, Rendering::IPhysicsObject ** objects, size_t numObjects);

		size_t MakeSubsets(Collision::DataStructures::BVHTree *node);
	private:

		void DrawRecursive(DataStructures::BVHTree *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		bool DescendDirection(DataStructures::BVHTree *left, DataStructures::BVHTree *right);

		std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> QueryBVHPairs(DataStructures::BVHTree *first, DataStructures::BVHTree *second);
		std::vector<Rendering::IPhysicsObject *> QueryBVH(DataStructures::BVHTree *queriedLeaf);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		// TODO see if performance is better if tree is kept as an array (left = 2 * i, right = 2 * i + 1)
		DataStructures::BVHTree *m_root;
	};
}
