#pragma once
#include "Scenario.h"

namespace Simulation
{
	class ScenarioGenerator
	{
	public:
		static std::vector<Simulation::Scenario> GenerateScenarios(int number);
		static void ExportScenarios(std::vector<Simulation::Scenario> &scens);
	private:
		static Simulation::ObjectDescription CreateDef(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale);
		static std::vector<Simulation::ObjectDescription> CreateDefsAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects);
	};
}