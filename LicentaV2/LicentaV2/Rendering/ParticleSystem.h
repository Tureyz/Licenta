#pragma once

#include "SceneObject.h"
#include "..\Core\TimeUtils.h"

namespace Managers
{
	class ModelManager;
	class MastersSimulationManager;
}

namespace Rendering
{

	class ParticleSystem
	{
	private:

		class Particle : public SceneObject
		{
		public:
			Particle(const glm::vec3 pos, const glm::vec3 vel, const TimeUtils::TimePointNano timeOfDeath);
			void FixedUpdate() override;
			glm::vec3 m_vel;
			TimeUtils::TimePointNano m_timeOfDeath;
		};

	public:
		ParticleSystem(const glm::vec3 center, const glm::vec2 radii, const glm::vec2 ttls, const size_t particleCount);
		void FixedUpdate();

		float m_minRadius;
		float m_maxRadius;
		glm::vec3 m_center;
		size_t m_particleCount;
		float m_minTTL;
		float m_maxTTL;
		std::vector<Particle *> m_particles;
		Managers::ModelManager *m_modelManager;
		Managers::MastersSimulationManager *m_simManager;
	private:

		Particle * SpawnParticle(const glm::vec3 pos, const glm::vec3 target, const float ttl);
	};
}
