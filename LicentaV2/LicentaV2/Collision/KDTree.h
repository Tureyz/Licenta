#pragma once
#include "ICollisionMethod.h"
#include "DataStructures\KDTreeNode.h"

namespace Collision
{
	class KDTree : public ICollisionMethod
	{
	public:
		KDTree(std::vector<Rendering::IPhysicsObject *> *allObjects, glm::vec3 worldMin, glm::vec3 worldMax);

		~KDTree();

		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		void SetDrawBuffers(GLuint vao, GLuint vbo, GLuint ibo);

		void SetParams(int splitThreshold, int maximumDepth);

	private:

		void InsertIntoTree(Rendering::IPhysicsObject *object);
		void __InsertIntoTree(DataStructures::KDTreeNode *node, Rendering::IPhysicsObject *object);

		void DrawRecursive(DataStructures::KDTreeNode *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);

		bool StraddleX(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node);
		bool StraddleY(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node);
		bool StraddleZ(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node);
		bool Straddle(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node);
		DataStructures::KDTreeNode *m_root;		
		glm::vec3 m_worldMin;
		glm::vec3 m_worldMax;
		GLuint m_vao, m_vbo, m_ibo;
	};
}
