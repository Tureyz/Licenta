#include "ScenarioGenerator.h"
#include "../Core/Utils.hpp"

std::vector<Simulation::Scenario> Simulation::ScenarioGenerator::GenerateScenarios(int number)
{
	std::vector<Simulation::Scenario> result;

	for (int i = 0; i < number; ++i)
	{
		auto defs = CreateDefsAround(glm::vec3(0.f, 0.f, 0.f), 1.0f, 50 * (i + 1), Simulation::PhysicsObjectType::OBJ_RANDOM);

		for (int i = 0; i < defs.size(); ++i)
		{
			defs[i].m_rotationStep = Core::Utils::RandomRange(0.000, 1.000);
			defs[i].m_rotationAngleStep = 0.00f + Core::Utils::RandomRange(0.00, 0.02).x;

			defs[i].m_translationStep = glm::normalize(defs[i].m_initialPosition) / 50.f;
		}

		

		Simulation::Scenario scen;
		scen.SetObjectDescriptions(defs);
		scen.m_name = "Scenario_" + std::to_string(i);
		scen.m_numberOfFrames = 90;
		result.push_back(scen);
	}

	return result;
}

void Simulation::ScenarioGenerator::ExportScenarios(std::vector<Simulation::Scenario> &scens)
{
	for (auto scen : scens)
	{
		scen.SaveToFile("SavedScenarios/" + scen.m_name + ".txt");
	}
}

Simulation::ObjectDescription Simulation::ScenarioGenerator::CreateDef(const Simulation::PhysicsObjectType objectType, size_t ID, const glm::vec3 &position, const glm::vec3 &rotation, const float rotationAngle, const glm::vec3 &scale)
{
	ObjectDescription result;

	result.m_ID = ID;
	result.m_initialPosition = position;
	result.m_initialRotation = rotation;
	result.m_initialRotationAngle = rotationAngle;
	result.m_initialScale = scale;
	result.m_objectType = objectType;
	result.m_rotationAngleStep = 0;
	result.m_rotationStep = glm::vec3(0.f);
	result.m_scaleStep = glm::vec3(1.f);
	result.m_translationStep = glm::vec3(0.f);

	return result;
}

std::vector<Simulation::ObjectDescription> Simulation::ScenarioGenerator::CreateDefsAround(const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects)
{
	size_t ID = 0;
	std::vector<Simulation::ObjectDescription> result;

	if (typeOfObjects == Simulation::PhysicsObjectType::OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			result.push_back(CreateDef((Simulation::PhysicsObjectType)(rand() % 3), ID++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRange(0.5f, 1.f)));
		}
	}
	else
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			result.push_back(CreateDef(typeOfObjects, ID++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRange(0.5f, 1.f)));
		}
	}

	return result;
}
