#pragma once
#include "ModelManager.h"
#include "../Collision/BVH.h"
#include "../Collision/KDTree.h"
#include "../Collision/Octree.h"
#include "../Collision/SpatialGrid.h"
#include "../Collision/SweepAndPrune.h"
#include "../Collision/SpatialHashing.h"
#include <unordered_map>

namespace Managers
{
	class SimulationManager
	{
	public:
		SimulationManager(ModelManager *modelManager);
		~SimulationManager();
		void Init();
		void Update();
		void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);
		void Draw();

		void KeyPressed(unsigned char key);
		void MousePressed(int button, int state, int x, int y);
		void MouseMove(int x, int y, int width, int height);

		void SpawnObjectAt(const glm::vec3 &position, const physicsObjectType objectType, const glm::vec4 &color);
		void SpawnManyAround(const glm::vec3 &position, const float radius, const int numberOfObjects, Managers::physicsObjectType typeOfObjects);

		glm::vec3 RandomPositionAround(const glm::vec3 &position, const float radius);
		glm::vec3 RandomRotationAxis();
		glm::vec3 RandomScale(float min, float max);

		void TestCollision();
		bool GetCollisionDebug() const { return m_collisionDebug; }
		void SetCollisionDebug(bool val) { m_collisionDebug = val; }
	private:

		void ResetCollisions();
		ModelManager *m_modelManager;
		//Collision::ICollisionMethod *m_activeMethod;
		std::unordered_map<std::string, Collision::ICollisionMethod *> m_collisionMethods;
		std::unordered_map<std::string, Collision::ICollisionMethod *>::iterator m_activeMethod;
		std::vector<IPhysicsObject*> *m_allObjects;
		size_t m_objectIDCounter;

		bool m_collisionDebug;

		int m_time;
		int m_timeBase;
	};
}