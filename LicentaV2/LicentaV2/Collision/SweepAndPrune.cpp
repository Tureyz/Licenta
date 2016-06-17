#include "SweepAndPrune.h"
#include "../Rendering/Models/Model.h"
#include <iterator>

Collision::SweepAndPrune::SweepAndPrune(std::vector<Rendering::IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
	m_memoryUsed = sizeof(Collision::SweepAndPrune);
}

Collision::SweepAndPrune::~SweepAndPrune()
{
	for (auto proj : m_xList)
	{
		delete proj;
		proj = NULL;
	}

	for (auto proj : m_yList)
	{
		delete proj;
		proj = NULL;
	}

	for (auto proj : m_zList)
	{
		delete proj;
		proj = NULL;
	}
}

void Collision::SweepAndPrune::_Update()
{
	m_lastFrameTests = 0;
	InsertionSort();
}

void Collision::SweepAndPrune::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
}

void Collision::SweepAndPrune::InsertionSort(std::vector<Projection *> &list)
{

	for (int i = 1; i < list.size(); ++i)
	{
		Projection *firstProj = list[i];
		float coord = firstProj->m_coord;

		int j = i - 1;

		while (j >= 0 && list[j]->m_coord > coord)
		{
			Projection *secondProj = list[j];

			if (firstProj->m_begin && !secondProj->m_begin)
			{
				m_lastFrameTests++;
				if (((Rendering::Models::Model *)firstProj->m_object)->GetBoundingBox()->Collides(((Rendering::Models::Model *)secondProj->m_object)->GetBoundingBox()))
				{
					m_collisionPairs.insert(std::make_pair(firstProj->m_object, secondProj->m_object));
					m_memoryUsed += sizeof(std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>);
				}
			}

			if (!firstProj->m_begin && secondProj->m_begin)
			{
				m_lastFrameTests++;
				if (m_collisionPairs.erase(std::make_pair(firstProj->m_object, secondProj->m_object)) == 1)
				{
					m_memoryUsed -= sizeof(std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>);
				}
			}

			list[j + 1] = secondProj;
			j--;
		}
		list[j + 1] = firstProj;
	}
}

void Collision::SweepAndPrune::ObjectMoved(Rendering::IPhysicsObject *object)
{
	ObjectProjectionPointers *ptrs = (ObjectProjectionPointers *)object->m_auxCollisionData;
	Collision::DataStructures::BoundingBox *bb = ((Rendering::Models::Model *)object)->GetBoundingBox();

	if (!ptrs)
	{
		ptrs = new ObjectProjectionPointers();
		m_memoryUsed += sizeof(ObjectProjectionPointers);
	}

	ptrs->m_xBegin->m_coord = bb->m_minX;
	ptrs->m_yBegin->m_coord = bb->m_minY;
	ptrs->m_zBegin->m_coord = bb->m_minZ;

	ptrs->m_xEnd->m_coord = bb->m_maxX;
	ptrs->m_yEnd->m_coord = bb->m_maxY;
	ptrs->m_zEnd->m_coord = bb->m_maxZ;

}

void Collision::SweepAndPrune::ObjectAdded(Rendering::IPhysicsObject *object)
{
	Rendering::Models::Model *castedObj = (Rendering::Models::Model *) object;
	Collision::DataStructures::BoundingBox *bb = castedObj->GetBoundingBox();

	Projection *xProjBegin = new Projection(bb->m_minX, true, object);
	Projection *xProjEnd = new Projection(bb->m_maxX, false, object);

	Projection *yProjBegin = new Projection(bb->m_minY, true, object);
	Projection *yProjEnd = new Projection(bb->m_maxY, false, object);

	Projection *zProjBegin = new Projection(bb->m_minZ, true, object);
	Projection *zProjEnd = new Projection(bb->m_maxZ, false, object);


	m_xList.push_back(xProjBegin);
	m_xList.push_back(xProjEnd);

	m_yList.push_back(yProjBegin);
	m_yList.push_back(yProjEnd);

	m_zList.push_back(zProjBegin);
	m_zList.push_back(zProjEnd);


	ObjectProjectionPointers *ptrs = new ObjectProjectionPointers(xProjBegin, xProjEnd, yProjBegin, yProjEnd, zProjBegin, zProjEnd);

	object->m_auxCollisionData = ptrs;

	m_memoryUsed += sizeof(Projection) * 6 + sizeof(ObjectProjectionPointers);
}

void Collision::SweepAndPrune::ObjectRemoved(Rendering::IPhysicsObject *object)
{

	for (auto it = m_xList.begin(); it != m_xList.end(); ++it)
	{
		if ((*it)->m_object == object)
		{
			it = m_xList.erase(it);
			m_memoryUsed -= sizeof(Projection);
		}
	}

	for (auto it = m_yList.begin(); it != m_yList.end(); ++it)
	{
		if ((*it)->m_object == object)
		{
			it = m_yList.erase(it);
			m_memoryUsed -= sizeof(Projection);
		}
	}

	for (auto it = m_zList.begin(); it != m_zList.end(); ++it)
	{
		if ((*it)->m_object == object)
		{
			it = m_zList.erase(it);
			m_memoryUsed -= sizeof(Projection);
		}
	}
}

void Collision::SweepAndPrune::InsertionSort()
{
	InsertionSort(m_xList);
	InsertionSort(m_yList);
	InsertionSort(m_zList);
}

std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::SweepAndPrune::_TestCollision()
{
	return m_collisionPairs;
}
