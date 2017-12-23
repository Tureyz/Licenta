#include "PhysicsManager.h"
#include "../Rendering/Models/Sphere.h"
#include <algorithm>

Managers::PhysicsManager::PhysicsManager(std::vector<Rendering::IPhysicsObject*> *objectList)
{
	m_objectList = objectList;

	m_defaultFrics = 0.4f;
	m_defaultGravMultiplier = 500000;
	m_gravStep = 500000;
	m_defaultRestitution = 0.5f;

	m_frics = m_defaultFrics;
	m_gravityMultiplier = m_defaultGravMultiplier;
	m_restitution = m_defaultRestitution;

	m_worldBounds = std::make_pair(glm::vec3(-50, -50, -50), glm::vec3(50, 50, 50));
	m_gravityCenter = glm::vec3(0); // only active if realGravity is false

	m_linearVelDecay = 0.995f;
	m_angularVelDecay = 0.998f;

	m_gravityVel = m_gravityMultiplier * m_gravitationalConstant;

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
					if (firstObj == secondObj)
						continue;

					//auto dist = std::fmaxf(0.01, glm::distance(firstObj->GetPosition(), secondObj->GetPosition()));
								

					glm::vec3 r12 = secondObj->GetPosition() - firstObj->GetPosition();

					float len = glm::length(r12);
					float dstSq = len * len;

					totalGravitationalPull += ((secondObj->GetMass()) / dstSq) * glm::normalize(r12);
				}

				firstObj->SetTranslationStep((firstObj->GetTranslationStep() + (m_gravityVel * totalGravitationalPull)) * m_linearVelDecay);
				firstObj->SetRotationStep(firstObj->GetRotationStep() * m_angularVelDecay);
				firstObj->SetRotationAngleStep(glm::length(firstObj->GetRotationStep()));
			}
		}
		else
		{
			for (auto obj : *m_objectList)
			{
				obj->SetRotationAngleStep(obj->GetRotationAngleStep() * m_angularVelDecay);
				//glm::vec3 gravitationalPull = model->GetMass() * (m_gravityCenter - model->GetPosition()) * m_gravityVel * (1.f / std::fmaxf(m_gravityVel, glm::distance(m_gravityCenter, model->GetPosition())));
				auto distanceToCenter = std::fmaxf(0.1f, glm::distance(obj->GetPosition(), m_gravityCenter));
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
		Rendering::Models::Sphere *firstObj = (Rendering::Models::Sphere *) pair.first;
		Rendering::Models::Sphere *secondObj = (Rendering::Models::Sphere *) pair.second;

		PushObjectsApart(firstObj, secondObj);
		
		auto res = ComputeReactionVels(firstObj, secondObj);

		glm::vec3 finalV1 = res.first.first;
		glm::vec3 finalOm1 = res.first.second;

		glm::vec3 finalV2 = res.second.first;
		glm::vec3 finalOm2 = res.second.second;

		ApplyAngularVel(firstObj, finalOm1);
		ApplyAngularVel(secondObj, finalOm2);

		ApplyLinearVel(firstObj, secondObj, finalV1);
		ApplyLinearVel(secondObj, firstObj, finalV2);		
	}

}

void Managers::PhysicsManager::KeyPressed(unsigned char key)
{
	
	switch (key)
	{
	case '1':
		m_gravityMultiplier = glm::clamp(glm::vec3(m_gravityMultiplier - m_gravStep), glm::vec3(1), glm::vec3(10000000)).x;
		m_gravityVel = m_gravityMultiplier * m_gravitationalConstant;
		std::wcout << L"Gravity multiplier is now " << m_gravityMultiplier << std::endl;
		break;
	case '2':
		m_gravityMultiplier = glm::clamp(glm::vec3(m_gravityMultiplier + (m_gravityMultiplier == 1 ? m_gravStep - 1 : m_gravStep)), glm::vec3(1), glm::vec3(10000000)).x;;
		m_gravityVel = m_gravityMultiplier * m_gravitationalConstant;
		std::wcout << L"Gravity multiplier is now " << m_gravityMultiplier << std::endl;
		break;
	case '3':
		m_frics = glm::clamp(glm::vec3(m_frics - 0.025f), glm::vec3(0), glm::vec3(1)).x;
		std::wcout << L"Friction coefficient is now " << m_frics << L" (0 = no friction, 1 = grip)" << std::endl;
		break;
	case '4':
		m_frics = glm::clamp(glm::vec3(m_frics + 0.025f), glm::vec3(0), glm::vec3(1)).x;
		std::wcout << L"Friction coefficient is now " << m_frics << L" (0 = no friction, 1 = grip)" << std::endl;
		break;
	case '5':
		m_restitution = glm::clamp(glm::vec3(m_restitution - 0.025f), glm::vec3(0), glm::vec3(1)).x;
		std::wcout << L"Restitution coefficient is now " << m_restitution << L" (0 = plastic, 1 = elastic)" << std::endl;
		break;
	case '6':
		m_restitution = glm::clamp(glm::vec3(m_restitution + 0.025f), glm::vec3(0), glm::vec3(1)).x;
		std::wcout << L"Restitution coefficient is now " << m_restitution << L" (0 = plastic, 1 = elastic)" << std::endl;
		break;
	case '`':
		std::wcout << L"Physics params have been reset to default" << std::endl;
		m_restitution = m_defaultRestitution;
		m_frics = m_defaultFrics;
		m_gravityMultiplier = m_defaultGravMultiplier;
		m_gravityVel = m_gravityMultiplier * m_gravitationalConstant;
		break;
	}
}

void Managers::PhysicsManager::KeyReleased(unsigned char key)
{

}

bool Managers::PhysicsManager::WillBreak(Rendering::IPhysicsObject * obj, Rendering::IPhysicsObject * other, glm::vec3 force)
{	
	float breakCoefficient = glm::length(force) * other->GetMass();
	float resistCoefficient = obj->GetSphereRadius() * obj->GetMass() / 5;

//  	if (obj->GetSphereRadius() > 4.f)
//  		std::wcout << breakCoefficient << L" " << resistCoefficient << std::endl;

	return (breakCoefficient > resistCoefficient) && obj->GetSphereRadius() > 1;
}

void Managers::PhysicsManager::PushObjectsApart(Rendering::Models::Sphere *firstObj, Rendering::Models::Sphere *secondObj)
{
	glm::vec3 firstCenter = firstObj->GetPosition();
	glm::vec3 secondCenter = secondObj->GetPosition();

	float firstRadius = firstObj->GetSphereRadius();
	float secondRadius = secondObj->GetSphereRadius();


	float firstMass = firstObj->GetMass();
	float secondMass = secondObj->GetMass();	

	float push = (firstRadius + secondRadius - glm::distance(firstCenter, secondCenter));

	float massSum = firstMass + secondMass;
	float firstPushFac = secondMass / massSum;
	float secondPushFac = firstMass / massSum;

	glm::vec3 n = glm::normalize(secondCenter - firstCenter);


	// was relative before
	firstObj->TranslateAbsolute(-n * push * firstPushFac);
	secondObj->TranslateAbsolute(n * push * secondPushFac);
}

std::pair<std::pair<glm::vec3, glm::vec3>, std::pair<glm::vec3, glm::vec3>> Managers::PhysicsManager::ComputeReactionVels(Rendering::Models::Sphere *firstObj, Rendering::Models::Sphere *secondObj)
{
	glm::vec3 firstCenter = firstObj->GetPosition();
	glm::vec3 secondCenter = secondObj->GetPosition();

	float firstRadius = firstObj->GetSphereRadius();
	float secondRadius = secondObj->GetSphereRadius();

	float m1 = firstObj->GetMass();
	float m2 = secondObj->GetMass();

	float im1 = firstObj->GetInverseMass();
	float im2 = secondObj->GetInverseMass();

	glm::mat3 invFirstTensor = firstObj->GetInverseInertiaTensor();
	glm::mat3 invSecondTensor = secondObj->GetInverseInertiaTensor();

	glm::vec3 n = glm::normalize(secondCenter - firstCenter);

	glm::vec3 contactPoint = firstCenter + n * firstObj->GetSphereRadius();

	glm::vec3 v1 = firstObj->GetTranslationStep();
	glm::vec3 v2 = secondObj->GetTranslationStep();

	glm::vec3 om1 = firstObj->GetRotationStep();
	glm::vec3 om2 = secondObj->GetRotationStep();

	glm::vec3 r1 = contactPoint - firstCenter;
	glm::vec3 r2 = contactPoint - secondCenter;

	glm::vec3 vp1 = v1 + glm::cross(om1, r1);
	glm::vec3 vp2 = v2 + glm::cross(om2, r2);

	glm::vec3 vr = vp2 - vp1;


	glm::vec3 fac1Resp = glm::cross(invFirstTensor * glm::cross(r1, n), r1);
	glm::vec3 fac2Resp = glm::cross(invSecondTensor * glm::cross(r2, n), r2);

	float upperSideResp = glm::dot(-(1 + m_restitution) * vr, n);
	float lowerSideResp = im1 + im2 + glm::dot(fac1Resp + fac2Resp, n);

	float jr = upperSideResp / lowerSideResp;
	glm::vec3 jrVec = n * jr;

	glm::vec3 t = glm::normalize(glm::cross(glm::cross(n, vr), n));

	glm::vec3 fac1Fric = glm::cross(invFirstTensor * glm::cross(r1, t), r1);
	glm::vec3 fac2Fric = glm::cross(invSecondTensor * glm::cross(r2, t), r2);

	float upperSideFric = glm::dot(-(1 + m_frics) * vr, t);
	float lowerSideFric = im1 + im2 + glm::dot(fac1Fric + fac2Fric, t);

	//float jf = upperSideFric / lowerSideFric;

	glm::vec3 jfVec = t * jr * m_frics;

	glm::vec3 finalV1 = v1 + (-jrVec + jfVec) * im1;
	glm::vec3 finalV2 = v2 +  (jrVec + jfVec) * im2;

	glm::vec3 finalOm1 = om1 + glm::cross(r1, -jrVec - jfVec) * invFirstTensor;
	glm::vec3 finalOm2 = om2 + glm::cross(r2,  jrVec + jfVec) * invSecondTensor;

	
	return std::make_pair(std::make_pair(finalV1, finalOm1), std::make_pair(finalV2, finalOm2));
}

void Managers::PhysicsManager::ApplyAngularVel(Rendering::Models::Sphere *obj, glm::vec3 axis)
{
	obj->SetRotationStep(axis);
	obj->SetRotationAngleStep(glm::length(axis));
}

void Managers::PhysicsManager::ApplyLinearVel(Rendering::Models::Sphere *firstObj, Rendering::Models::Sphere *secondObj, glm::vec3 force)
{
	if (!firstObj->GetBroken())
	{
		if (WillBreak(firstObj, secondObj, force))
		{
			//std::wcout << L"B";
			firstObj->GetSimulationManager()->BreakObject(firstObj, force);
		}
		else
		{
			firstObj->SetTranslationStep(force);
		}
	}
}
