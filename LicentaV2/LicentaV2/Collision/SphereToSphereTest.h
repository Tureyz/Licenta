#pragma once
#include "ICollisionMethod.h"

namespace Collision
{
	class SphereToSphereTest : public ICollisionMethod
	{

	public:
		SphereToSphereTest(std::vector<Rendering::IPhysicsObject *> *allObjects);

		virtual void DrawDebug(const glm::mat4& viewProjection) override;


		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;


		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;


		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

	protected:
		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;


		virtual void _Update() override;

	};
}