#pragma once
#include "ICollisionMethod.h"
#include <map>

namespace Collision
{
	class SpatialGrid : public ICollisionMethod
	{
	public:
		SpatialGrid(std::vector<Rendering::IPhysicsObject *> *allObjects, int numberOfCells);

		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

		void SetParams(glm::vec3 worldMin, glm::vec3 worldMax, int numberOfCells);


	private:
		std::vector<glm::vec3> FindCells(Rendering::IPhysicsObject *obj);

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;


		glm::vec3 m_worldMin;
		glm::vec3 m_worldMax;
		int m_numberOfCells;
		glm::vec3 m_cellSize;
		std::vector<std::vector<std::vector<std::vector<Rendering::IPhysicsObject *>>>> m_grid;
		
	};
}
