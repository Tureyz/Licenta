#pragma once
#include "ICollisionMethod.h"
#include <unordered_set>

namespace Collision
{
	class SeparatingAxisTheorem : public ICollisionMethod
	{
	public:

		SeparatingAxisTheorem(std::vector<Rendering::SceneObject *> *allObjects);

		~SeparatingAxisTheorem();
		
		virtual void DrawDebug(const glm::mat4& viewProjection) override;

		virtual void ObjectMoved(Rendering::SceneObject *object) override;

		virtual void ObjectAdded(Rendering::SceneObject *object) override;

		virtual void ObjectRemoved(Rendering::SceneObject *object) override;

	protected:

		virtual std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		std::vector<glm::vec3> GetTestingAxes(Rendering::SceneObject *obj1, Rendering::SceneObject *obj2);

		std::pair<float, float> GetProjection(Rendering::SceneObject *obj, const glm::vec3 &axis);		

		bool TestTwoObjects(Rendering::SceneObject *obj1, Rendering::SceneObject *obj2);

		bool TestProjectionOverlap(const std::pair<float, float> &proj1, const std::pair<float, float> &proj2);

	};
}