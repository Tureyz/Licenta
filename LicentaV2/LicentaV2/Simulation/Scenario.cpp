#include "Scenario.h"
#include <iterator>
#include <sstream>
#include <algorithm>

using namespace std;

void Simulation::Scenario::LoadFromObjects(std::vector<Rendering::IPhysicsObject*> objects, std::string scenarioName, size_t numberOfFrames)
{
	m_objectDescriptions.clear();
	this->m_name = scenarioName;
	this->m_numberOfFrames = numberOfFrames;
	for (auto obj : objects)
	{
		ObjectDescription objDesc;

		objDesc.m_objectType = (PhysicsObjectType) obj->GetObjectType();
		objDesc.m_ID = obj->GetID();
		objDesc.m_initialPosition = obj->GetPosition();
		objDesc.m_initialRotation = obj->GetRotation();
		objDesc.m_initialRotationAngle = obj->GetRotationAngle();
		objDesc.m_initialScale = obj->GetScale();
		objDesc.m_rotationAngleStep = obj->GetRotationAngleStep();
		objDesc.m_rotationStep = obj->GetRotationStep();
		objDesc.m_scaleStep = obj->GetScaleStep();
		objDesc.m_translationStep = obj->GetTranslationStep();

		m_objectDescriptions.push_back(objDesc);
	}
}

void Simulation::Scenario::LoadFromFile(std::string fileName)
{
	ifstream file(fileName);
	if (!file.is_open())
	{
		std::cout << "ERROR OPENING FILE\n";
	}
	string line;
	getline(file, m_name);

	getline(file, line);

	m_numberOfFrames = std::stoull(line);
	m_objectDescriptions.clear();

	while (getline(file, line))
	{
		std::stringstream strstr(line);

		std::istream_iterator<std::string> it(strstr);
		std::istream_iterator<std::string> end;
		std::vector<std::string> results(it, end);

		ObjectDescription objDesc;
		int i = 0;
		objDesc.m_objectType = (Simulation::PhysicsObjectType) std::stoi(results[i++]);
		objDesc.m_ID = std::stoull(results[i++]);

		objDesc.m_initialPosition.x = std::stof(results[i++]);
		objDesc.m_initialPosition.y = std::stof(results[i++]);
		objDesc.m_initialPosition.z = std::stof(results[i++]);

		objDesc.m_initialRotation.x = std::stof(results[i++]);
		objDesc.m_initialRotation.y = std::stof(results[i++]);
		objDesc.m_initialRotation.z = std::stof(results[i++]);

		objDesc.m_initialRotationAngle = std::stof(results[i++]);

		objDesc.m_initialScale.x = std::stof(results[i++]);
		objDesc.m_initialScale.y = std::stof(results[i++]);
		objDesc.m_initialScale.z = std::stof(results[i++]);

		objDesc.m_rotationAngleStep = std::stof(results[i++]);

		objDesc.m_rotationStep.x = std::stof(results[i++]);
		objDesc.m_rotationStep.y = std::stof(results[i++]);
		objDesc.m_rotationStep.z = std::stof(results[i++]);

		objDesc.m_scaleStep.x = std::stof(results[i++]);
		objDesc.m_scaleStep.y = std::stof(results[i++]);
		objDesc.m_scaleStep.z = std::stof(results[i++]);

		objDesc.m_translationStep.x = std::stof(results[i++]);
		objDesc.m_translationStep.y = std::stof(results[i++]);
		objDesc.m_translationStep.z = std::stof(results[i++]);

		m_objectDescriptions.push_back(objDesc);
	}

	file.close();
}

void Simulation::Scenario::SaveToFile(std::string fileName)
{
	ofstream file(fileName);

	if (!file.is_open())
	{
		std::cout << "ERROR OPENING FILE\n";
	}
	file << m_name << "\n";
	file << m_numberOfFrames << "\n";

	for (auto objDesc : m_objectDescriptions)
	{
		file << objDesc.m_objectType << " ";
		file << objDesc.m_ID << " ";
		file << objDesc.m_initialPosition.x << " " << objDesc.m_initialPosition.y << " " << objDesc.m_initialPosition.z << " ";
		file << objDesc.m_initialRotation.x << " " << objDesc.m_initialRotation.y << " " << objDesc.m_initialRotation.z << " ";
		file << objDesc.m_initialRotationAngle << " ";
		file << objDesc.m_initialScale.x << " " << objDesc.m_initialScale.y << " " << objDesc.m_initialScale.z << " ";
		file << objDesc.m_rotationAngleStep << " ";
		file << objDesc.m_rotationStep.x << " " << objDesc.m_rotationStep.y << " " << objDesc.m_rotationStep.z << " ";
		file << objDesc.m_scaleStep.x << " " << objDesc.m_scaleStep.y << " " << objDesc.m_scaleStep.z << " ";
		file << objDesc.m_translationStep.x << " " << objDesc.m_translationStep.y << " " << objDesc.m_translationStep.z << "\n";
	}

	file.close();
}

std::vector<Simulation::ObjectDescription> Simulation::Scenario::GetDescriptionsByFrame(int frameNumber)
{
	std::vector<Simulation::ObjectDescription> result(m_objectDescriptions);

	for (int i = 0; i < frameNumber; ++i)
	{
		for (auto desc : result)
		{
			desc.m_initialPosition += desc.m_translationStep;
			desc.m_initialRotation += desc.m_rotationStep;
			desc.m_initialRotationAngle += desc.m_rotationAngleStep;
			desc.m_initialScale += desc.m_scaleStep;
		}
	}

	return result;
}
