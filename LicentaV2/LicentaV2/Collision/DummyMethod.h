#pragma once
#include "ICollisionMethod.h"

namespace Collision
{
	class DummyMethod : public ICollisionMethod
	{
	public:
		DummyMethod(std::vector<IPhysicsObject *> *allObjects);
		virtual ~DummyMethod();

		virtual std::vector<IPhysicsObject *> TestCollision(IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;


	};
}