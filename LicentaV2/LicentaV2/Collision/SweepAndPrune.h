#pragma once
#include "ICollisionMethod.h"
#include "DataStructures/LinkedList.hpp"
#include <unordered_set>

namespace Collision
{
	class SweepAndPrune : public ICollisionMethod
	{
	public:
		SweepAndPrune(std::vector<Rendering::IPhysicsObject *> *allObjects);
		~SweepAndPrune();

		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) override;

		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() override;

		virtual void Update() override;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;
	private:

		struct Projection
		{
			float m_coord;
			bool m_begin;
			Rendering::IPhysicsObject *m_object;
		};

		struct ObjectProjectionPointers
		{
			Projection *m_xBegin, *m_xEnd;
			Projection *m_yBegin, *m_yEnd;
			Projection *m_zBegin, *m_zEnd;
		};

		void InsertionSort(std::vector<Projection *> &list);
		void InsertionSort();

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;


		//DataStructures::LinkedList<Projection> m_xList, m_yList, m_zList;
		std::vector<Projection *> m_xList, m_yList, m_zList;

		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> m_collisionPairs;
	};
}