#pragma once

#include "ISimulationManager.h"
#include "../Collision/BVH.h"
#include "../Collision/NarrowBVH.h"


namespace Managers
{
	class MastersSimulationManager : public ISimulationManager
	{
	public:

		MastersSimulationManager(Managers::ModelManager *modelManager);

		virtual void Init() override;

		virtual void FixedUpdate() override;

		virtual void Update() override;

		virtual void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		virtual void Draw() override;

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		virtual void KeyPressed(unsigned char key) override;

		virtual void KeyReleased(unsigned char key) override;

		virtual void MousePressed(int button, int state, int x, int y) override;

		virtual void MouseMove(int x, int y, int width, int height) override;

		virtual void BreakObject(Rendering::IPhysicsObject *obj, glm::vec3 impactForce) override;

	private:

		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> GetBroadPhasePairs();
		Collision::ICollisionMethod *m_broadPhaseMethod;

		std::unordered_map<size_t, Collision::NarrowBVH *> m_narrowMethods;
		bool m_objectBBsVisible;
		bool m_broadPhaseDebugDraw;
		bool m_narrowPhaseDebugDraw;
	};
}