#pragma once
#include "ICollisionMethod.h"

namespace Collision
{
	class DummyMethod : public ICollisionMethod
	{
	public:
		DummyMethod(std::vector<Rendering::SceneObject *> *allObjects);

		virtual ~DummyMethod();

		virtual void _Update() override;

		virtual void DrawDebug(const glm::mat4& viewProjection) override;

		virtual void ObjectMoved(Rendering::SceneObject *object) override;

		virtual void ObjectAdded(Rendering::SceneObject *object) override;

		virtual void ObjectRemoved(Rendering::SceneObject *object) override;

	protected:
		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() override;

	};
}