#pragma once
#include "ModelManager.h"
#include "../Collision/BVH.h"
#include "../Collision/KDTree.h"
#include "../Collision/Octree.h"
#include "../Collision/SpatialGrid.h"
#include "../Collision/SweepAndPrune.h"
#include "../Collision/SpatialHashing.h"
#include "../Collision/DummyMethod.h"
#include <unordered_map>
#include "../Simulation/Scenario.h"

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

		void ObjectMoved(IPhysicsObject *object);
		void ObjectAdded(IPhysicsObject *object);
		void ObjectRemoved(IPhysicsObject *object);

		void KeyPressed(unsigned char key);
		void MousePressed(int button, int state, int x, int y);
		void MouseMove(int x, int y, int width, int height);

		Rendering::IPhysicsObject* SpawnObjectAt(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale);
		void SpawnManyAround(const glm::vec3 &position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects);
		void SpawnObjectsFromScenario(const Simulation::Scenario *scenario);

		void TestCollision();
		bool GetCollisionDebug() const { return m_collisionDebug; }
		void SetCollisionDebug(bool val) { m_collisionDebug = val; }

		void BenchmarkAllScenarios();
		void ExportCurrentScenarioResults();

		void ImportAllAvailableScenarios();

		void InitCollisionMethods();
		void FreeCollisionMethods();
	private:

		void ResetCollisions();
		void LoadScenario(Simulation::Scenario *scenario);
		void CleanupCurrentScenario();
		ModelManager *m_modelManager;
		//Collision::ICollisionMethod *m_activeMethod;
		std::unordered_map<std::string, Collision::ICollisionMethod *> m_collisionMethods;
		std::unordered_map<std::string, Collision::ICollisionMethod *>::iterator m_activeMethod;
		std::vector<IPhysicsObject*> *m_allObjects;
		size_t m_objectIDCounter;

		bool m_collisionDebug;
		bool m_objectBBs;

		int m_time;
		int m_timeBase;

		const glm::vec4 m_defaultObjectColor = glm::vec4(0.7f, 0.7f, 0.7f, 1.f);

		bool m_runningBenchmark;
		size_t m_currentSimulationFrame;

		std::vector<Simulation::Scenario *> m_scenarios;
		Simulation::Scenario *m_activeScenario;

		size_t m_maxSimulationFrame;
		int m_currentScenarioIndex;

		struct PerFrameCriteria
		{
			std::unordered_map<std::string, float> Element;
		};

		struct MethodFrameResult
		{
			std::unordered_map<std::string, PerFrameCriteria> Element;
		};

		std::vector<MethodFrameResult> m_benchmarkPerFrameResults;
	};
}