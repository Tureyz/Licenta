#include "PhysicsManager.h"
#include "../Rendering/Models/Sphere.h"

Managers::PhysicsManager::PhysicsManager(std::vector<Rendering::IPhysicsObject*> *objectList)
{
	m_objectList = objectList;
	m_linearVelDecay = 0.994f;
	m_angularVelDecay = 0.994f;
	m_gravityCenter = glm::vec3(0);
	m_worldBounds = std::make_pair(glm::vec3(-50, -50, -50), glm::vec3(50, 50, 50));
	m_gravityVel = 0.00005f;
	m_gravityToggle = true;
	m_realGravity = true;
	m_worldBoundToggle = true;

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

				firstObj->SetTranslationStep(firstObj->GetTranslationStep() + (m_gravityVel * totalGravitationalPull));
				firstObj->SetRotationAngleStep(firstObj->GetRotationAngleStep() * m_angularVelDecay);
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
	if (!m_worldBoundToggle)
		return;

	for (auto obj : *m_objectList)
	{
		auto objPos = obj->GetPosition();
		auto objRad = obj->GetSphereRadius();
		auto objTr = obj->GetTranslationStep() * 0.99f;

		if (objPos.x < m_worldBounds.first.x + objRad)
		{
			obj->SetTranslationStep(glm::reflect(objTr, glm::vec3(1, 0, 0)));
		}
		else if (objPos.x > m_worldBounds.second.x - objRad)
		{
			obj->SetTranslationStep(glm::reflect(objTr, glm::vec3(-1, 0, 0)));
		}
		if (objPos.y < m_worldBounds.first.y + objRad)
		{
			obj->SetTranslationStep(glm::reflect(objTr, glm::vec3(0, 1, 0)));
		}
		else if (objPos.y > m_worldBounds.second.y - objRad)
		{
			obj->SetTranslationStep(glm::reflect(objTr, glm::vec3(0, -1, 0)));
		}
		if (objPos.z < m_worldBounds.first.z + objRad)
		{
			obj->SetTranslationStep(glm::reflect(objTr, glm::vec3(0, 0, 1)));
		}
		else if (objPos.z > m_worldBounds.second.z - objRad)
		{
			obj->SetTranslationStep(glm::reflect(objTr, glm::vec3(0, 0, -1)));
		}
	}
}

static void printVec(glm::vec3 input)
{
	std::wcout << L"x: " << input.x << L", y: " << input.y << L", z: " << input.z << std::endl;
}

void Managers::PhysicsManager::CollisionResponse()
{
	for (std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> pair : *m_collisionPairs)
	{
		float restitution = 0.75f;

		Rendering::Models::Sphere *firstObj = (Rendering::Models::Sphere *) pair.first;
		Rendering::Models::Sphere *secondObj = (Rendering::Models::Sphere *) pair.second;

		glm::vec3 firstCenter = firstObj->GetPosition();
		glm::vec3 secondCenter = secondObj->GetPosition();

		float firstRadius = firstObj->GetSphereRadius();
		float secondRadius = secondObj->GetSphereRadius();

		// Push objects apart so they only touch


		float push = (firstRadius + secondRadius - glm::distance(firstCenter, secondCenter)) / 2.f;

		glm::vec3 contactNormal = glm::normalize(secondCenter - firstCenter);

		firstObj->TranslateRelative(-contactNormal * push);
		secondObj->TranslateRelative(contactNormal * push);

		// Recalculate centers
		firstCenter = firstObj->GetPosition();
		secondCenter = secondObj->GetPosition();

		float firstMass = firstObj->GetMass();
		float secondMass = secondObj->GetMass();

		float im1 = 1.f / firstMass;
		float im2 = 1.f / secondMass;

		glm::mat3 invFirstTensor = glm::mat3(1) * (1.f / (0.4f * firstMass * firstRadius * firstRadius));
		glm::mat3 invSecondTensor = glm::mat3(1) * (1.f / (0.4f * secondMass * secondRadius * secondRadius));		

		contactNormal = glm::normalize(secondCenter - firstCenter);
		glm::vec3 contactPoint = firstCenter + contactNormal * firstObj->GetSphereRadius();

		glm::vec3 v1 = firstObj->GetTranslationStep();
		glm::vec3 v2 = secondObj->GetTranslationStep();

		glm::vec3 om1 = firstObj->GetRotationStep();
		glm::vec3 om2 = secondObj->GetRotationStep();

		glm::vec3 r1 = contactPoint - firstCenter;
		glm::vec3 r2 = contactPoint - secondCenter;

		glm::vec3 vp1 = v1 + glm::cross(om1, r1);
		glm::vec3 vp2 = v2 + glm::cross(om2, r2);

		glm::vec3 vr = vp2 - vp1;

		glm::vec3 fac1 = glm::cross(invFirstTensor * glm::cross(r1, contactNormal), r1);
		glm::vec3 fac2 = glm::cross(invSecondTensor * glm::cross(r2, contactNormal), r2);

		float upperSide = glm::dot(-(1 + restitution) * vr, contactNormal);
		float lowerSide = im1 + im2 + glm::dot(fac1 + fac2, contactNormal);

		float jr = upperSide / lowerSide;
		
		glm::vec3 jrVec = contactNormal * jr;

		glm::vec3 finalV1 = v1 - jrVec * im1;
		glm::vec3 finalV2 = v2 + jrVec * im2;

		glm::vec3 finalOm1 = om1 - jr * invFirstTensor * glm::cross(r1, contactNormal);
		glm::vec3 finalOm2 = om2 + jr * invSecondTensor * glm::cross(r2, contactNormal);

		firstObj->SetRotationStep(finalOm1);
		secondObj->SetRotationStep(finalOm2);

		firstObj->SetRotationAngleStep(glm::length(finalOm1) / 10);
		secondObj->SetRotationAngleStep(glm::length(finalOm2) / 10);


		if (!firstObj->GetBroken())
		{
			if (WillBreak(firstObj, secondObj, finalV1))
			{
				firstObj->GetSimulationManager()->BreakObject(firstObj, finalV1);
			}
			else
			{
				firstObj->SetTranslationStep(finalV1);
			}
		}

		if (!secondObj->GetBroken())
		{
			if (WillBreak(secondObj, firstObj, finalV2))
			{
				secondObj->GetSimulationManager()->BreakObject(secondObj, finalV2);
			}
			else
			{
				secondObj->SetTranslationStep(finalV2);
			}
		}
	}

// 	for (std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> pair : *m_collisionPairs)
// 	{
// 		Rendering::Models::Sphere *firstObj = (Rendering::Models::Sphere *) pair.first;
// 		Rendering::Models::Sphere *secondObj = (Rendering::Models::Sphere *) pair.second;
// 
// 		glm::vec3 firstCenter = firstObj->GetPosition();
// 		glm::vec3 secondCenter = secondObj->GetPosition();
// 
// 		float firstMass = firstObj->GetMass();
// 		float secondMass = secondObj->GetMass();
// 
// 		glm::vec3 delta = firstCenter - secondCenter;
// 
// 		float restitution = 0.8f;		
// 		float massSum = firstMass + secondMass;
// 
// 		glm::vec3 n = glm::normalize(delta);
// 
// 		float a1 = glm::dot(firstObj->GetTranslationStep(), n);
// 		float a2 = glm::dot(secondObj->GetTranslationStep(), n);
// 
// 		float p = (2.0 * (a1 - a2)) / massSum;
// 
// 		// Compute their impulse
// 
// 		glm::vec3 firstImpulse = firstObj->GetTranslationStep() - p * secondObj->GetMass() * n;
// 		glm::vec3 secondImpulse = secondObj->GetTranslationStep() + p * firstObj->GetMass() * n;
// 		float ua = glm::length(firstObj->GetTranslationStep());
// 		float ub = glm::length(secondObj->GetTranslationStep());
// 
// 		float va = glm::length(firstObj->GetTranslationStep() - p * secondObj->GetMass() * n);
// 		float vb = glm::length(secondObj->GetTranslationStep() + p * firstObj->GetMass() * n);
// 
// 		float massVel = firstMass * ua + secondMass * ub;
// 		float velDif = ub - ua;
// 
// 		glm::vec3 finalFirstImpulse = glm::normalize(firstImpulse) * ((massVel + secondMass * restitution * velDif) / massSum);
// 		glm::vec3 finalSecondImpulse = glm::normalize(secondImpulse) * ((massVel + firstMass * restitution * -velDif) / massSum);
//  
// 		if (firstObj->GetSphereRadius() > 4 && secondObj->GetSphereRadius() > 4)
// 			std::wcout << secondMass / firstMass << L" " << glm::length(firstImpulse) * firstObj->GetDensity() / 2 << std::endl;
// 	
// 
// 		//std::wcout << glm::length(firstImpulse) * secondMass / firstMass << L", " << firstObj->GetSphereRadius() << L" " << glm::length(secondImpulse) * firstMass / secondMass << L", " << secondObj->GetSphereRadius() << std::endl;
// 		if (!firstObj->GetBroken())
// 		{
// 			if (WillBreak(firstObj, secondObj, firstImpulse))
// 			{
// 				firstObj->GetSimulationManager()->BreakObject(firstObj, finalFirstImpulse);
// 			}
// 			else
// 			{
// 				firstObj->SetTranslationStep(finalFirstImpulse);
// 			}
// 		}
// 
// 		if (!secondObj->GetBroken())
// 		{
// 			if (WillBreak(secondObj, firstObj, secondImpulse))
// 			{
// 				secondObj->GetSimulationManager()->BreakObject(secondObj, finalSecondImpulse);
// 			}
// 			else
// 			{
// 				secondObj->SetTranslationStep(finalSecondImpulse);
// 			}
// 		}
// 	}

}

bool Managers::PhysicsManager::WillBreak(Rendering::IPhysicsObject * obj, Rendering::IPhysicsObject * other, glm::vec3 force)
{	
	//std::wcout << other->GetMass() / obj->GetMass() << L" " << glm::length(force) * obj->GetDensity() << std::endl;

	float breakCoefficient = other->GetMass() / obj->GetMass();
	float resistCoefficient = glm::length(force) * obj->GetDensity() / 2.1;

	return (breakCoefficient > resistCoefficient) && obj->GetSphereRadius() > 1;
}
