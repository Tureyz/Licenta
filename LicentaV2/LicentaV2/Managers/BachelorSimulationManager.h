#pragma once
#include <unordered_map>

#include "ISimulationManager.h"

#include "../Collision/BVH.h"
#include "../Collision/Octree.h"
#include "../Collision/SpatialGrid.h"
#include "../Collision/SpatialGridOptimized.h"
#include "../Collision/SweepAndPrune.h"
#include "../Collision/SpatialHashing.h"
#include "../Collision/DummyMethod.h"
#include "../Simulation/Scenario.h"
#include "BenchmarkResultManager.h"



namespace Managers
{
	class BachelorSimulationManager : public ISimulationManager
	{
	public:
		BachelorSimulationManager(ModelManager *modelManager);
		~BachelorSimulationManager();
		void Init();
		void FixedUpdate();
		void Update();
		void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix);
		void Draw();

		void ObjectMoved(SceneObject *object);
		void ObjectAdded(SceneObject *object);
		void ObjectRemoved(SceneObject *object);

		void KeyPressed(unsigned char key);
		void KeyReleased(unsigned char key);
		void MousePressed(int button, int state, int x, int y);
		void MouseMove(int x, int y, int width, int height);

		void BreakObject(Rendering::SceneObject *obj, glm::vec3 impactForce);

		std::unordered_set<std::pair<SceneObject *, SceneObject *>> *GetCurrentCollisionPairsPtr() { return &m_currentCollisionPairs; }
		void SetCurrentCollisionPairs(std::unordered_set<std::pair<SceneObject *, SceneObject *>> &val) { m_currentCollisionPairs = val; }

	private:
		void SpawnObjectsFromScenario(const Simulation::Scenario *scenario);

		void TestCollision();
		bool GetCollisionDebug() const { return m_collisionDebug; }
		void SetCollisionDebug(bool val) { m_collisionDebug = val; }

		void BenchmarkAllScenarios();

		void ImportAllAvailableScenarios();

		void InitCollisionMethods();
		void FreeCollisionMethods();


		void DebugBreakAll();

		//Rendering::IPhysicsObject* SpawnObjectAt(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale);
		//void SpawnManyAround(const glm::vec3 &position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects);
		void ResetCollisions();
		void LoadScenario(Simulation::Scenario *scenario);
		void CleanupCurrentScenario();

		void RecordLastFrameResults();
		void CurrentScenarioEnded();

		void DisplayPercentComplete();

		void DisplayHelp();

		void CreateWorldBounds();
		std::wstring  PercentFinished();


		std::unordered_map<std::wstring , Collision::ICollisionMethod *> m_collisionMethods;
		std::unordered_map<std::wstring , Collision::ICollisionMethod *>::iterator m_activeMethod;
		
		

		bool m_collisionDebug;
		bool m_objectBBs;
		bool m_simulationDebug;

		int m_time;
		int m_timeBase;		

		bool m_runningBenchmark;
		size_t m_currentSimulationFrame;

		std::vector<std::vector<Simulation::Scenario *>> m_scenarios;
		Simulation::Scenario *m_activeScenario;

		size_t m_maxSimulationFrame;
		std::pair<int, int> m_currentScenarioIndex;

// 		struct PerFrameCriteria
// 		{
// 			std::unordered_map<std::wstring , float> Element; // <criterion name, value>
// 		};
// 
// 		struct MethodFrameResult
// 		{
// 			std::unordered_map<std::wstring , PerFrameCriteria> Element; // <method name, all criteria values>
// 		};
// 
// 		std::vector<MethodFrameResult> m_benchmarkPerFrameResults;
// 		std::vector<MethodFrameResult> m_benchmarkPerScenarioResults;

		Managers::BenchmarkResultManager m_resultManager;
		std::wstring  m_lastActiveMethodName;

		const std::wstring  m_defaultMethodName = Core::METHOD_OCTREE;

		std::chrono::time_point<std::chrono::steady_clock> m_benchmarkStartTime;

		std::wstring  m_benchmarkTimeInfo;

		std::unordered_set<std::pair<SceneObject *, SceneObject *>> m_currentCollisionPairs;

		size_t m_firstAvailableID;

		std::vector<Simulation::ObjectDescription> m_addQueue;
		std::vector<Rendering::SceneObject*> m_removeQueue;

		std::pair<glm::vec3, glm::vec3> m_worldBounds;
		std::vector<std::pair<float, glm::vec3>> m_worldNormals;

		const float m_childSphereCoef = 0.40421356237f;
		
	};
}