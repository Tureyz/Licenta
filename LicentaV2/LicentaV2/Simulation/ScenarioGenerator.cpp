#include "ScenarioGenerator.h"
#include "../Core/Utils.hpp"

std::vector<Simulation::Scenario> Simulation::ScenarioGenerator::GenerateScenarios(int number)
{
	size_t numberOfObjects = 150;
	size_t numberOfFrames = 300;
	
	std::vector<Simulation::Scenario> result;
	result.push_back(CreateStaticScenario(0, numberOfFrames, numberOfObjects, 10.f));
	result.push_back(CreateSingleMovingObjectScenario(1, numberOfFrames, numberOfObjects, 7.f, glm::vec3(8, 8, 8)));
	result.push_back(CreateManyMovingObjectsScenario(2, numberOfFrames, numberOfObjects, 7.f, 9.f));
	result.push_back(CreateFrontalCrashScenario(3, numberOfFrames, numberOfObjects, 4.f, 8));
	result.push_back(CreateExplosionScenario(4, numberOfFrames, numberOfObjects, 1.f));

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
	size_t index = 0;
	return Simulation::ScenarioGenerator::CreateDefsAround(index, position, radius, numberOfObjects, typeOfObjects);
}

std::vector<Simulation::ObjectDescription> Simulation::ScenarioGenerator::CreateDefsAround(size_t &startingIndex, const glm::vec3 & position, const float radius, const int numberOfObjects, Simulation::PhysicsObjectType typeOfObjects)
{
	size_t ID = startingIndex;
	std::vector<Simulation::ObjectDescription> result;

	if (typeOfObjects == Simulation::PhysicsObjectType::OBJ_RANDOM)
	{
		for (int i = 0; i < numberOfObjects; ++i)
		{
			glm::vec3 pos = Core::Utils::RandomVec3Around(position, radius);
			result.push_back(CreateDef((Simulation::PhysicsObjectType)(rand() % Simulation::PhysicsObjectType::OBJ_NUM_TOTAL), ID++, pos, Core::Utils::Random01(), (float)(std::rand() % 360), Core::Utils::RandomRange(0.5f, 1.f)));
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

	startingIndex = ID;
	return result;
}

Simulation::Scenario Simulation::ScenarioGenerator::CreateStaticScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius)
{
	auto defs = CreateDefsAround(glm::vec3(0.f, 0.f, 0.f), radius, numberOfObjects, Simulation::PhysicsObjectType::OBJ_RANDOM);

	return GenerateScenarioFromStats(defs, scenarioID, numberOfFrames);
}

Simulation::Scenario Simulation::ScenarioGenerator::CreateSingleMovingObjectScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius, glm::vec3 movingObjectPosition)
{
	auto defs = CreateDefsAround(glm::vec3(0.f, 0.f, 0.f), radius, numberOfObjects, Simulation::PhysicsObjectType::OBJ_RANDOM);

	defs[0].m_initialPosition = movingObjectPosition;
	defs[0].m_rotationStep = Core::Utils::RandomRange(0.000, 1.000);
	defs[0].m_rotationAngleStep = 0.00f + Core::Utils::RandomRange(0.00, 2.00).x / 50.f;
	defs[0].m_translationStep = -glm::normalize(defs[0].m_initialPosition) * 2.f * glm::distance(movingObjectPosition, glm::vec3(0)) / (float)(numberOfFrames);

	return GenerateScenarioFromStats(defs, scenarioID, numberOfFrames);
}

Simulation::Scenario Simulation::ScenarioGenerator::CreateManyMovingObjectsScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius, float movingObjectsRadius)
{
	auto defs = CreateDefsAround(glm::vec3(0.f, 0.f, 0.f), radius, numberOfObjects, Simulation::PhysicsObjectType::OBJ_RANDOM);

	for (int i = 0; i < 0.25 * (float) (defs.size()); ++i)
	{
		defs[i].m_initialPosition = Core::Utils::RandomVec3Around(glm::vec3(0), movingObjectsRadius - radius) * glm::vec3(radius, radius, radius);
		defs[i].m_rotationStep = Core::Utils::RandomRange(0.000, 1.000);
		defs[i].m_rotationAngleStep = 0.00f + Core::Utils::RandomRange(0.00, 2.00).x / 50.f;
		defs[i].m_translationStep = -glm::normalize(defs[i].m_initialPosition) * 1.f * glm::distance(defs[i].m_initialPosition, glm::vec3(0)) / (float)(numberOfFrames);
	}

	return GenerateScenarioFromStats(defs, scenarioID, numberOfFrames);
}

Simulation::Scenario Simulation::ScenarioGenerator::CreateFrontalCrashScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius, float clusterDistance)
{	
	size_t index = 0;
	auto defs1 = CreateDefsAround(index, glm::vec3(-clusterDistance, 0.f, 0.f), radius, numberOfObjects / 2, Simulation::PhysicsObjectType::OBJ_RANDOM);
	auto defs2 = CreateDefsAround(index, glm::vec3(clusterDistance, 0.f, 0.f), radius, numberOfObjects / 2, Simulation::PhysicsObjectType::OBJ_RANDOM);

	for (int i = 0; i < defs1.size(); ++i)
	{
		defs1[i].m_rotationStep = Core::Utils::RandomRange(0.000, 1.000);
		defs1[i].m_rotationAngleStep = 0.00f + Core::Utils::RandomRange(0.00, 2.00).x / 50.f;
		defs2[i].m_rotationStep = Core::Utils::RandomRange(0.000, 1.000);
		defs2[i].m_rotationAngleStep = 0.00f + Core::Utils::RandomRange(0.00, 2.00).x / 50.f;

		defs1[i].m_translationStep = (defs2[i].m_initialPosition - defs1[i].m_initialPosition) / (float)(numberOfFrames);
		defs2[i].m_translationStep = (defs1[i].m_initialPosition - defs2[i].m_initialPosition) / (float)(numberOfFrames);
	}

	defs1.insert(defs1.end(), defs2.begin(), defs2.end());

	return GenerateScenarioFromStats(defs1, scenarioID, numberOfFrames);
}

Simulation::Scenario Simulation::ScenarioGenerator::CreateExplosionScenario(int scenarioID, int numberOfFrames, int numberOfObjects, float radius)
{
	auto defs = CreateDefsAround(glm::vec3(0.f, 0.f, 0.f), radius, numberOfObjects, Simulation::PhysicsObjectType::OBJ_RANDOM);

	for (int i = 0; i < defs.size(); ++i)
	{
		defs[i].m_rotationStep = Core::Utils::RandomRange(0.000, 1.000);
		defs[i].m_rotationAngleStep = 0.00f + Core::Utils::RandomRange(0.00, 4.00).x / 50.f;

		defs[i].m_translationStep = Core::Utils::RandomRange(0.00, 2.00).x * glm::normalize(defs[i].m_initialPosition) * 7.f * radius / (float)numberOfFrames;
	}

	return GenerateScenarioFromStats(defs, scenarioID, numberOfFrames);
}

Simulation::Scenario Simulation::ScenarioGenerator::GenerateScenarioFromStats(std::vector<ObjectDescription> &descriptions, size_t ID, size_t numberOfFrames)
{
	Simulation::Scenario scen;
	scen.SetObjectDescriptions(descriptions);
	scen.m_name = "Scenario_" + std::to_string(ID);
	scen.m_numberOfFrames = numberOfFrames;

	return scen;
}
