#include "ParticleSystem.h"
#include "../Managers/ModelManager.h"
#include "../Managers/MastersSimulationManager.h"
#include "../Dependencies/glm/gtx/rotate_vector.hpp"

Rendering::ParticleSystem::ParticleSystem(const glm::vec3 center, const glm::vec2 radii, const glm::vec2 ttls, const size_t particleCount)
{
	m_center = center;
	m_minRadius = radii.x;
	m_maxRadius = radii.y;
	m_minTTL = ttls.x;
	m_maxTTL = ttls.y;
	m_particleCount = particleCount;
}

void Rendering::ParticleSystem::FixedUpdate()
{
	for (auto it = m_particles.begin(); it != m_particles.end();)
	{
		if (TimeUtils::Now() > (*it)->m_timeOfDeath)
		{
			m_modelManager->DeleteModel((*it)->GetID());
			it = m_particles.erase(it);
		}
		else
		{
			++it;
		}
	}

	while (m_particles.size() < m_particleCount)
	{
		glm::vec3 dest = m_center + Core::Utils::RandomRangeVec(-m_maxRadius, m_maxRadius);
		float ttl = Core::Utils::RandomRange(m_minTTL, m_maxTTL);
		Particle *part = SpawnParticle(m_center, dest, ttl);
		m_particles.push_back(part);		
	}
}

Rendering::ParticleSystem::Particle * Rendering::ParticleSystem::SpawnParticle(const glm::vec3 pos, const glm::vec3 target, const float ttl)
{
		glm::vec3 vel = Core::TIME_STEP * (target - pos) / ttl;

		Particle *result = new Particle(pos, vel, TimeUtils::Now() + std::chrono::nanoseconds(static_cast<long long>(ttl * 1000000000)));
		result->SetID(m_simManager->nextID());
		result->SetVisualBody(Rendering::VisualBodyFactory::GetInstance().CreateBasicVisualBody(Rendering::VisualBodyType::OBJ_CUBE));
		result->RotateAbsolute(Core::Utils::Random01(), Core::Utils::RandomRange(0, 6));
		result->ScaleAbsolute(glm::vec3(0.07f));
		result->TranslateAbsolute(pos);
// 		result->SetRotationStep(Core::Utils::RandomRangeVec(-1, 1));
// 		result->SetRotationAngleStep(Core::Utils::RandomRange(0, 0.01));
		result->SetTranslationStep(vel);
		result->Update();
		m_modelManager->RegisterSFXObject(result->GetID(), result);

		return result;
}

Rendering::ParticleSystem::Particle::Particle(const glm::vec3 pos, const glm::vec3 vel, const TimeUtils::TimePointNano timeOfDeath)
{
	m_position = pos;
	m_vel = vel;
	m_timeOfDeath = timeOfDeath;
}

void Rendering::ParticleSystem::Particle::FixedUpdate()
{
	
}
