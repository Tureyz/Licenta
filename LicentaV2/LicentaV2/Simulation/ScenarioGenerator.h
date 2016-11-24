#pragma once
#include "Scenario.h"

namespace Simulation
{
	class ScenarioGenerator
	{
	public:
		static std::vector<Simulation::Scenario> GenerateScenarios(int number);
		static void ExportScenarios(std::vector<Simulation::Scenario> &scens);
		static Simulation::ObjectDescription CreateDef(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale);
	private:
		static std::vector<Simulation::ObjectDescription> CreateDefsAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects);
		static std::vector<Simulation::ObjectDescription> CreateDefsAround(size_t &startingIndex, const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects);

		static Simulation::Scenario CreateStaticScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius);
		static Simulation::Scenario CreateSingleMovingObjectScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius, glm::vec3 movingObjectPosition);
		static Simulation::Scenario CreateManyMovingObjectsScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius, float movingObjectsRadius);
		static Simulation::Scenario CreateFrontalCrashScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius, float clusterDistance);
		static Simulation::Scenario CreateExplosionScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius);
		static Simulation::Scenario CreateScreenshotScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float distance);

		static Simulation::Scenario GenerateScenarioFromStats(std::vector<ObjectDescription> &descriptions, size_t ID, size_t numberOfFrames);
	};
}