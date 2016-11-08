#include "PhysicsManager.h"
#include "../Rendering/Models/Sphere.h"

Managers::PhysicsManager::PhysicsManager(std::vector<Rendering::IPhysicsObject*> *objectList)
{
	m_objectList = objectList;
	m_linearVelDecay = 0.994f;
	m_angularVelDecay = 0.994f;
	m_gravityCenter = glm::vec3(0);
	m_gravityVel = 0.005f;
	m_gravityToggle = true;
	m_realGravity = true;
}

void Managers::PhysicsManager::FixedUpdate()
{
	if (m_gravityToggle)
	{
		if (m_realGravity)
		{
			for (auto firstObj : *m_objectList)
			{
				glm::vec3 totalGravitationalPull(0);
				for (auto secondObj : *m_objectList)
				{
					auto dist = std::fmaxf(0.01, glm::distance(firstObj->GetPosition(), secondObj->GetPosition()));
					totalGravitationalPull += (secondObj->GetPosition() - firstObj->GetPosition()) * ((secondObj->GetMass())/ (dist * dist));
				}

				firstObj->SetTranslationStep(firstObj->GetTranslationStep() + (0.0001f * totalGravitationalPull));
			}
		}
		else
		{
			for (auto obj : *m_objectList)
			{
				obj->SetRotationAngleStep(obj->GetRotationAngleStep() * m_angularVelDecay);
				//glm::vec3 gravitationalPull = model->GetMass() * (m_gravityCenter - model->GetPosition()) * m_gravityVel * (1.f / std::fmaxf(m_gravityVel, glm::distance(m_gravityCenter, model->GetPosition())));
				auto distanceToCenter = std::fmaxf(0.1, glm::distance(obj->GetPosition(), m_gravityCenter));
				glm::vec3 gravitationalPull = obj->GetMass() * m_gravityVel * (m_gravityCenter - obj->GetPosition()) / (distanceToCenter * distanceToCenter);
				obj->SetTranslationStep(obj->GetTranslationStep() + gravitationalPull);
			}
		}
	}
	else
	{
		for (auto model : *m_objectList)
		{
			model->SetTranslationStep(model->GetTranslationStep() * m_linearVelDecay);
		}
	}

}

void Managers::PhysicsManager::Update()
{

}

static void printVec(glm::vec3 input)
{
	std::wcout << L"x: " << input.x << L", y: " << input.y << L", z: " << input.z << std::endl;
}

void Managers::PhysicsManager::CollisionResponse()
{
	for (std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> pair : *m_collisionPairs)
	{
		Rendering::Models::Sphere *firstObj = (Rendering::Models::Sphere *) pair.first;
		Rendering::Models::Sphere *secondObj = (Rendering::Models::Sphere *) pair.second;

 		glm::vec3 firstCenter = firstObj->GetPosition();
 		glm::vec3 secondCenter = secondObj->GetPosition();
// 		if (glm::distance(firstCenter, secondCenter) < 0) {
// 			std::wcout << "a";
// 		}



		float firstRadius = firstObj->GetSphereRadius();
		float secondRadius = secondObj->GetSphereRadius();
		if (glm::distance(firstCenter, secondCenter) > firstRadius + secondRadius)
		{
			continue;
		}
		glm::vec3 delta = firstCenter - secondCenter;
		float d = glm::length(delta);

		glm::vec3 mtd = delta * (((firstRadius + secondRadius) - d) / d);
		float im1 = 1.0 / firstObj->GetMass();
		float im2 = 1.0 / secondObj->GetMass();

		glm::vec3 translationFirst = (mtd * (im1 / (im1 + im2)));
		glm::vec3 translationSecond = -(mtd * (im2 / (im1 + im2)));
		// 		printVec(translationFirst);
		// 		printVec(translationSecond);
		firstObj->TranslateRelative(translationFirst);
		secondObj->TranslateRelative(translationSecond);
		glm::vec3 v = firstObj->GetTranslationStep() - secondObj->GetTranslationStep();

		glm::vec3 normalizedMtd = glm::length(mtd) < 0.0001 ? glm::vec3(0) : glm::normalize(mtd);
		float vn = glm::dot(v, normalizedMtd);
		if (vn >= 0.0f)
		{
			continue;
		}

		float i = (-(1.0f + 7.5f) * vn) / (im1 + im2);
		glm::vec3 impulse = mtd * i;
		firstObj->SetTranslationStep(firstObj->GetTranslationStep() + (impulse * im1));
		secondObj->SetTranslationStep(secondObj->GetTranslationStep() - (impulse * im2));

	}

}
