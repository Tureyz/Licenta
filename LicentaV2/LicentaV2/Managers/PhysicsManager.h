#pragma once
#include <vector>
#include "../Rendering/SceneObject.h"
#include "../Collision/ICollisionMethod.h"
#include <unordered_set>

namespace Managers
{
	class PhysicsManager
	{
	public:
		PhysicsManager(std::vector<Rendering::SceneObject*> *objectList);
		void FixedUpdate();
		void Update();
		void CollisionResponse();
		std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> *GetCollisionPairs() const { return m_collisionPairs; }
		void SetCollisionPairs(std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> *val) { m_collisionPairs = val; }

		void KeyPressed(unsigned char key);
		void KeyReleased(unsigned char key);


	private:
		bool WillBreak(Rendering::SceneObject * obj, Rendering::SceneObject * other, glm::vec3 force);


		void PushObjectsApart(Rendering::SceneObject *firstObj, Rendering::SceneObject *secondObj);

		std::pair<std::pair<glm::vec3, glm::vec3>, std::pair<glm::vec3, glm::vec3>> ComputeReactionVels(Rendering::SceneObject *firstObj, Rendering::SceneObject *secondObj);

		void ApplyAngularVel(Rendering::SceneObject *obj, glm::vec3 axis);

		void ApplyLinearVel(Rendering::SceneObject *firstObj, Rendering::SceneObject *secondObj, glm::vec3 force);

		std::vector<Rendering::SceneObject*> *m_objectList;
		std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> *m_collisionPairs;

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