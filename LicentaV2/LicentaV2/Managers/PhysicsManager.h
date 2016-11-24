#pragma once
#include <vector>
#include "../Rendering/IPhysicsObject.h"
#include "../Collision/ICollisionMethod.h"
#include <unordered_set>

namespace Managers
{
	class PhysicsManager
	{
	public:
		PhysicsManager(std::vector<Rendering::IPhysicsObject*> *objectList);
		void FixedUpdate();
		void Update();
		void CollisionResponse();
		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> *GetCollisionPairs() const { return m_collisionPairs; }
		void SetCollisionPairs(std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> *val) { m_collisionPairs = val; }
	private:

		bool WillBreak(Rendering::IPhysicsObject * obj, Rendering::IPhysicsObject * other, glm::vec3 force);
		std::vector<Rendering::IPhysicsObject*> *m_objectList;
		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> *m_collisionPairs;

		glm::vec3 m_gravityCenter;
		float m_gravityVel;
		bool m_gravityToggle;
		bool m_realGravity;
		float m_linearVelDecay;
		float m_angularVelDecay;

		bool m_worldBoundToggle;

		std::pair<glm::vec3, glm::vec3> m_worldBounds;

	};
}