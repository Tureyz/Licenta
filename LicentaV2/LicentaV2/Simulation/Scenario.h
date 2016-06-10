#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "..\Rendering\IPhysicsObject.h"

namespace Simulation
{
	enum PhysicsObjectType { OBJ_CUBE = 0, OBJ_SPHERE = 1, OBJ_TETRAHEDRON = 2, OBJ_CYLINDER = 3, OBJ_CONE = 4, OBJ_RANDOM , OBJ_NUM_TOTAL = 5};

	struct ObjectDescription
	{
		PhysicsObjectType m_objectType;
		size_t m_ID;

		glm::vec3 m_initialPosition;
		glm::vec3 m_initialRotation;
		float m_initialRotationAngle;
		glm::vec3 m_initialScale;

		glm::vec3 m_translationStep;
		glm::vec3 m_scaleStep;
		glm::vec3 m_rotationStep;
		float m_rotationAngleStep;
	};

	class Scenario
	{
	public:
		void LoadFromObjects(std::vector<Rendering::IPhysicsObject*> objects, std::string scenarioName, size_t numberOfFrames);
		void LoadFromFile(std::string fileName);
		void LoadFromFile(std::string fileName, int numberOfObjects);
		void SaveToFile(std::string fileName);
		std::vector<ObjectDescription> GetObjectDescriptions() const { return m_objectDescriptions; }
		std::vector<Simulation::ObjectDescription> GetDescriptionsByFrame(int frameNumber);
		void SetObjectDescriptions(std::vector<ObjectDescription> val) { m_objectDescriptions = val; }
		size_t m_numberOfFrames;
		std::string m_name;
	private:

		std::vector<ObjectDescription> m_objectDescriptions;
	};
}
