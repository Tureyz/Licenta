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
		std::vector<Rendering::IPhysicsObject*> *m_objectList;
		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> *m_collisionPairs;

		glm::vec3 m_gravityCenter;
		float m_gravityVel;
		bool m_gravityToggle;
		
		float m_linearVelDecay;
		float m_angularVelDecay;
	};
}