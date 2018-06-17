#pragma once

#include "ISimulationManager.h"
#include "../Collision/BVH.h"
#include "../Collision/NarrowBVH.h"


namespace Rendering
{
	class ParticleSystem;
}

namespace Managers
{
	class MastersSimulationManager : public ISimulationManager
	{
	public:

		MastersSimulationManager(Managers::ModelManager *modelManager);

		virtual void Init() override;

		virtual void FixedUpdate() override;

		virtual void Update() override;

		virtual void Draw(const glm::mat4& viewProjection) override;

		virtual void Draw() override;

		virtual void ObjectMoved(Rendering::SceneObject *object) override;

		virtual void ObjectAdded(Rendering::SceneObject *object) override;

		virtual void ObjectRemoved(Rendering::SceneObject *object) override;

		virtual void KeyPressed(unsigned char key) override;

		virtual void KeyReleased(unsigned char key) override;

		virtual void MousePressed(int button, int state, int x, int y) override;

		virtual void MouseMove(int x, int y, int width, int height) override;

		virtual void BreakObject(Rendering::SceneObject *obj, glm::vec3 impactForce) override;

		size_t nextID();

	private:

		std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> GetBroadPhasePairs();
		Collision::ICollisionMethod *m_broadPhaseMethod;
		std::unordered_map<size_t, Collision::NarrowBVH *> m_narrowMethods;
		bool m_objectBBsVisible;
		bool m_broadPhaseDebugDraw;
		bool m_narrowPhaseDebugDraw;

		Rendering::ParticleSystem *m_ps;

		Rendering::SceneObject *m_meshObj;
		Rendering::SceneObject *m_sphereObj;
	};
}