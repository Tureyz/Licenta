#pragma once
#include <vector>
#include "../Rendering/IPhysicsObject.h"
#include "../Rendering/Models/Sphere.h"
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

		void KeyPressed(unsigned char key);
		void KeyReleased(unsigned char key);


	private:
		bool WillBreak(Rendering::IPhysicsObject * obj, Rendering::IPhysicsObject * other, glm::vec3 force);


		void PushObjectsApart(Rendering::Models::Sphere *firstObj, Rendering::Models::Sphere *secondObj);

		std::pair<std::pair<glm::vec3, glm::vec3>, std::pair<glm::vec3, glm::vec3>> ComputeReactionVels(Rendering::Models::Sphere *firstObj, Rendering::Models::Sphere *secondObj);

		void ApplyAngularVel(Rendering::Models::Sphere *obj, glm::vec3 axis);

		void ApplyLinearVel(Rendering::Models::Sphere *firstObj, Rendering::Models::Sphere *secondObj, glm::vec3 force);

		std::vector<Rendering::IPhysicsObject*> *m_objectList;
		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> *m_collisionPairs;

		glm::vec3 m_gravityCenter;
		float m_gravityMultiplier;
		float m_gravityVel;
		bool m_gravityToggle;
		bool m_realGravity;
		float m_linearVelDecay;
		float m_angularVelDecay;

		bool m_worldBoundToggle;

		std::pair<glm::vec3, glm::vec3> m_worldBounds;


		float m_frics;
		float m_restitution;


		float m_defaultFrics;
		float m_defaultGravMultiplier;
		float m_defaultRestitution;

		const float m_gravitationalConstant = 6.674f * static_cast<float>(std::pow(10, -11));
		float m_gravStep;
	};
}