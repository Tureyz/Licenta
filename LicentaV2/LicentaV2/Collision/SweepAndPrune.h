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

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) override;

	protected:

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() override;

		virtual void _Update() override;

	private:

		class Projection
		{
		public:
			Projection(float coord, bool begin, Rendering::IPhysicsObject *object) : m_coord(coord), m_begin(begin), m_object(object) {}

			float m_coord;
			bool m_begin;
			Rendering::IPhysicsObject *m_object;
		};

		struct ObjectProjectionPointers
		{
		public:
			ObjectProjectionPointers() {}
			ObjectProjectionPointers(Projection *xb, Projection *xe, Projection *yb, Projection *ye, Projection *zb, Projection *ze) :
			m_xBegin(xb), m_xEnd(xe), m_yBegin(yb), m_yEnd(ye), m_zBegin(zb), m_zEnd(ze) {}

			Projection *m_xBegin, *m_xEnd;
			Projection *m_yBegin, *m_yEnd;
			Projection *m_zBegin, *m_zEnd;
		};

		void InsertionSort(std::vector<Projection *> &list);
		void InsertionSort();

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) override;

		virtual void ObjectAdded(Rendering::IPhysicsObject *object) override;

		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) override;

		std::vector<Projection *> m_xList, m_yList, m_zList;

		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> m_collisionPairs;
	};
}