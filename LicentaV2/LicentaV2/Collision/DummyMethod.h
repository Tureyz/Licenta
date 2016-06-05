#pragma once
#include "ICollisionMethod.h"

namespace Collision
{
	class DummyMethod : public ICollisionMethod
	{
	public:
		DummyMethod(std::vector<Rendering::IPhysicsObject *> *allObjects);

		virtual ~DummyMethod();

		virtual void _Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

	protected:
		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;

	};
}