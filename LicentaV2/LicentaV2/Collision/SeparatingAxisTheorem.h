#pragma once
#include "ICollisionMethod.h"
#include <unordered_set>

namespace Collision
{
	class SeparatingAxisTheorem : public ICollisionMethod
	{
	public:

		SeparatingAxisTheorem(std::vector<Rendering::IPhysicsObject *> *allObjects);

		~SeparatingAxisTheorem();
		
		virtual void DrawDebug(const glm::mat4& viewProjection) override;

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

	protected:

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		std::vector<glm::vec3> GetTestingAxes(Rendering::IPhysicsObject *obj1, Rendering::IPhysicsObject *obj2);

		std::pair<float, float> GetProjection(Rendering::IPhysicsObject *obj, const glm::vec3 &axis);		

		bool TestTwoObjects(Rendering::IPhysicsObject *obj1, Rendering::IPhysicsObject *obj2);

		bool TestProjectionOverlap(const std::pair<float, float> &proj1, const std::pair<float, float> &proj2);

	};
}