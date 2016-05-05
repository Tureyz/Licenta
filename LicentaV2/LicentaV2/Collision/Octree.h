#pragma once
#include "ICollisionMethod.h"
#include "DataStructures/OctreeNode.h"

namespace Collision
{
	class Octree : public ICollisionMethod
	{


	public:
		Octree(std::vector<Rendering::IPhysicsObject *> *allObjects, glm::vec3 worldMin, glm::vec3 worldMax);
		~Octree();
		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) override;
		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() override;
		virtual void Update() override;
		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;
		void SetParams(int splitThreshold, int maximumDepth);		

	private:

		void InsertIntoTree(Rendering::IPhysicsObject *object);
		void __InsertIntoTree(DataStructures::OctreeNode *node, Rendering::IPhysicsObject *object);

		bool CompletelyInside(Rendering::IPhysicsObject *object, glm::vec3 center, float halfWidth);
		bool StraddleX(Rendering::IPhysicsObject *object, DataStructures::OctreeNode *node);
		bool StraddleY(Rendering::IPhysicsObject *object, DataStructures::OctreeNode *node);
		bool StraddleZ(Rendering::IPhysicsObject *object, DataStructures::OctreeNode *node);

		void DrawRecursive(DataStructures::OctreeNode *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		void TestCollisionsRecursive(DataStructures::OctreeNode *node, std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> &collisions);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;


		DataStructures::OctreeNode *m_root;
		glm::vec3 m_worldCenter;
		float m_worldHalfW;

		int m_splitThreshold;
		int m_maximumDepth;


		
	};
}
