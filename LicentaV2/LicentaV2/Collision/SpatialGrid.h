#pragma once
#include "ICollisionMethod.h"
#include <map>

namespace Collision
{
	class SpatialGrid : public ICollisionMethod
	{
	public:

		SpatialGrid(std::vector<Rendering::IPhysicsObject *> *allObjects, int numberOfCells);


		virtual void DrawDebug(const glm::mat4& viewProjection) override;

		void SetParams(glm::vec3 worldMin, glm::vec3 worldMax, int numberOfCells);

	protected:

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:
		std::vector<glm::vec3> FindCells(Rendering::IPhysicsObject *obj);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		void MakeNewGrid();

		glm::vec3 m_worldMin;
		glm::vec3 m_worldMax;
		int m_numberOfCells;
		glm::vec3 m_cellSize;
		std::vector<std::vector<std::vector<std::vector<Rendering::IPhysicsObject *>>>> m_grid;
		
	};
}
