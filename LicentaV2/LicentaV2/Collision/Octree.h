#pragma once
#include "ICollisionMethod.h"
#include "DataStructures/OctreeNode.h"

namespace Collision
{
	class Octree : public ICollisionMethod
	{
	public:

		Octree(std::vector<Rendering::SceneObject *> *allObjects, glm::vec3 worldMin, glm::vec3 worldMax);
		~Octree();
		virtual void DrawDebug(const glm::mat4& viewProjection) override;
		void SetParams(int splitThreshold, int maximumDepth);		

	protected:

		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		void InsertIntoTree(Rendering::SceneObject *object);
		void __InsertIntoTree(DataStructures::OctreeNode *node, Rendering::SceneObject *object, int &depth);

		bool CompletelyInside(Rendering::SceneObject *object, glm::vec3 center, float halfWidth);
		bool StraddleX(Rendering::SceneObject *object, DataStructures::OctreeNode *node);
		bool StraddleY(Rendering::SceneObject *object, DataStructures::OctreeNode *node);
		bool StraddleZ(Rendering::SceneObject *object, DataStructures::OctreeNode *node);

		void DrawRecursive(DataStructures::OctreeNode *node, const glm::mat4& viewProjection);

		void TestCollisionsRecursive(DataStructures::OctreeNode *node, std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> &collisions);

		virtual void ObjectMoved(Rendering::SceneObject *object) override;

		virtual void ObjectAdded(Rendering::SceneObject *object) override;

		virtual void ObjectRemoved(Rendering::SceneObject *object) override;


		DataStructures::OctreeNode *m_root;
		glm::vec3 m_worldCenter;
		float m_worldHalfW;

		int m_splitThreshold;
		int m_maximumDepth;


		
	};
}
