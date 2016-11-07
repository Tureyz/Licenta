#include "PhysicsManager.h"
#include "../Rendering/Models/Sphere.h"

Managers::PhysicsManager::PhysicsManager(std::vector<Rendering::IPhysicsObject*> *objectList)
{
	m_objectList = objectList;
	m_linearVelDecay = 0.994f;
	m_angularVelDecay = 0.994f;
	m_gravityCenter = glm::vec3(0);
	m_gravityVel = 0.0003f;
	m_gravityToggle = true;
}

void Managers::PhysicsManager::FixedUpdate()
{
	for (auto model : *m_objectList)
	{
		model->SetRotationAngleStep(model->GetRotationAngleStep() * m_angularVelDecay);

		if (m_gravityToggle)
		{
			glm::vec3 gravitationalPull = model->GetMass() * (m_gravityCenter - model->GetPosition()) * m_gravityVel * (1.f / std::fmaxf(m_gravityVel, glm::distance(m_gravityCenter, model->GetPosition())));
			model->SetTranslationStep(model->GetTranslationStep() + gravitationalPull);
		}
		else
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

	for (auto pair : *m_collisionPairs)
	{		
		Rendering::Models::Sphere *firstObj = (Rendering::Models::Sphere *) pair.first;
		Rendering::Models::Sphere *secondObj = (Rendering::Models::Sphere *) pair.second;

		glm::vec3 firstCenter = firstObj->GetPosition();
		glm::vec3 secondCenter = secondObj->GetPosition();

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
	}

	for (auto pair : *m_collisionPairs)
	{
		Rendering::Models::Sphere *firstObj = (Rendering::Models::Sphere *) pair.first;
		Rendering::Models::Sphere *secondObj = (Rendering::Models::Sphere *) pair.second;

		auto firstCenter = firstObj->GetPosition();
		auto secondCenter = secondObj->GetPosition();

		auto firstRadius = firstObj->GetSphereRadius();
		auto secondRadius = secondObj->GetSphereRadius();

		auto delta = firstCenter - secondCenter;
		auto d = glm::length(delta);

		auto mtd = delta * (((firstRadius + secondRadius) - d) / d);

		auto v = firstObj->GetTranslationStep() - secondObj->GetTranslationStep();
		float vn = glm::dot(v, glm::normalize(mtd));

		if (vn > 0.0f)
		{
			continue;
		}

		float im1 = 1.0 / firstObj->GetMass();
		float im2 = 1.0 / secondObj->GetMass();
		float i = (-(1.0f + 1.0f) * vn) / (im1 + im2);
		std::wcout << L"i: " << i << L", vn: " << vn << L" -- ";
		printVec(mtd);
		glm::vec3 impulse = mtd * i;
		firstObj->SetTranslationStep(firstObj->GetTranslationStep() + (impulse * im1));
		secondObj->SetTranslationStep(secondObj->GetTranslationStep() - (impulse * im2));
	}
}
