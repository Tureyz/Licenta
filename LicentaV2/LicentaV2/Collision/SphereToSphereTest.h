#pragma once
#include "ICollisionMethod.h"

namespace Collision
{
	class SphereToSphereTest : public ICollisionMethod
	{

	public:
		SphereToSphereTest(std::vector<Rendering::SceneObject *> *allObjects);

		virtual void DrawDebug(const glm::mat4& viewProjection) override;


		virtual void ObjectMoved(Rendering::SceneObject *object) override;


		virtual void ObjectAdded(Rendering::SceneObject *object) override;


		virtual void ObjectRemoved(Rendering::SceneObject *object) override;

	protected:
		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() override;


		virtual void _Update() override;

	};
}