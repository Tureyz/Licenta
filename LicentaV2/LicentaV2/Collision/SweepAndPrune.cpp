#include "SweepAndPrune.h"
#include "../Rendering/Models/Model.h"
#include <iterator>

Collision::SweepAndPrune::SweepAndPrune(std::vector<Rendering::IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;

// 	for (auto obj : *m_allObjects)
// 	{
// 		
// 	}
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

std::vector<Rendering::IPhysicsObject *> Collision::SweepAndPrune::TestCollision(Rendering::IPhysicsObject *queriedObject)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::SweepAndPrune::Update()
{
	auto start = std::chrono::high_resolution_clock::now();

	InsertionSort();

	auto end = std::chrono::high_resolution_clock::now();

	auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	m_lastFrameCriteria["Time Spent - Structure Update"] = (float) timeSpent;
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
			//std::cout << "SWAP\n";
			Projection *secondProj = list[j];

			if (firstProj->m_begin && !secondProj->m_begin)
			{
				m_lastFrameComparisons++;
				if (((Rendering::Models::Model *)firstProj->m_object)->GetBoundingBox()->Collides(((Rendering::Models::Model *)secondProj->m_object)->GetBoundingBox()))
				{
					//std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> fst = std::make_pair(firstProj.m_object, secondProj.m_object);
					//std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> snd = std::make_pair(secondProj.m_object, firstProj.m_object);

					m_collisionPairs.insert(std::make_pair(firstProj->m_object, secondProj->m_object));
					//m_collisionPairs.insert(snd);
				}
			}

			if (!firstProj->m_begin && secondProj->m_begin)
			{
				//std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> fst = std::make_pair(firstProj.m_object, secondProj.m_object);
				//std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> snd = std::make_pair(secondProj.m_object, firstProj.m_object);

// 				auto it = std::find(m_collisionPairs.begin(), m_collisionPairs.end(), fst);
// 				if (it != m_collisionPairs.end())
// 					m_collisionPairs.erase(it);
// 
// 				it = std::find(m_collisionPairs.begin(), m_collisionPairs.end(), snd);
// 				if (it != m_collisionPairs.end())
// 					m_collisionPairs.erase(it);

				m_collisionPairs.erase(std::make_pair(firstProj->m_object, secondProj->m_object));
				//m_collisionPairs.erase(snd);
			}

			list[j + 1] = secondProj;
			j--;
		}
		list[j + 1] = firstProj;
	}

// 	for (auto asd : list)
// 	{
// 		std::cout << asd.m_coord << " ";
// 
// 	}
}

void Collision::SweepAndPrune::ObjectMoved(Rendering::IPhysicsObject *object)
{
	ObjectProjectionPointers *ptrs = (ObjectProjectionPointers *)object->m_auxCollisionData;
	Collision::DataStructures::BoundingBox *bb = ((Rendering::Models::Model *)object)->GetBoundingBox();
	if (!ptrs)
	{
		ptrs = new ObjectProjectionPointers();
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
	Rendering::Models::Model *castedObj = (Rendering::Models::Model *)object;

	Projection *xProjBegin = new Projection();
	xProjBegin->m_coord = castedObj->GetBoundingBox()->m_minX;
	xProjBegin->m_begin = true;
	xProjBegin->m_object = object;

	Projection *xProjEnd = new Projection();
	xProjEnd->m_coord = castedObj->GetBoundingBox()->m_maxX;
	xProjEnd->m_begin = false;
	xProjEnd->m_object = object;

	//m_xList.InsertBack(xProjBegin);
	//m_xList.InsertBack(xProjEnd);
	m_xList.push_back(xProjBegin);
	m_xList.push_back(xProjEnd);

	Projection *yProjBegin = new Projection();
	yProjBegin->m_coord = castedObj->GetBoundingBox()->m_minY;
	yProjBegin->m_begin = true;
	yProjBegin->m_object = object;

	Projection *yProjEnd = new Projection();
	yProjEnd->m_coord = castedObj->GetBoundingBox()->m_maxY;
	yProjEnd->m_begin = false;
	yProjEnd->m_object = object;

	// 		m_yList.InsertBack(yProjBegin);
	// 		m_yList.InsertBack(yProjEnd);
	m_yList.push_back(yProjBegin);
	m_yList.push_back(yProjEnd);

	Projection *zProjBegin = new Projection();
	zProjBegin->m_coord = castedObj->GetBoundingBox()->m_minZ;
	zProjBegin->m_begin = true;
	zProjBegin->m_object = object;

	Projection *zProjEnd = new Projection();
	zProjEnd->m_coord = castedObj->GetBoundingBox()->m_maxZ;
	zProjEnd->m_begin = false;
	zProjEnd->m_object = object;

	// 		m_zList.InsertBack(zProjBegin);
	// 		m_zList.InsertBack(zProjEnd);
	m_zList.push_back(zProjBegin);
	m_zList.push_back(zProjEnd);

	ObjectProjectionPointers *ptrs = new ObjectProjectionPointers();
	ptrs->m_xBegin = xProjBegin;
	ptrs->m_xEnd = xProjEnd;
	ptrs->m_yBegin = yProjBegin;
	ptrs->m_yEnd = yProjEnd;
	ptrs->m_zBegin = zProjBegin;
	ptrs->m_zEnd = zProjEnd;

	object->m_auxCollisionData = ptrs;
}

void Collision::SweepAndPrune::ObjectRemoved(Rendering::IPhysicsObject *object)
{

	for (auto it = m_xList.begin(); it != m_xList.end(); ++it)
	{
		if ((*it)->m_object == object)
		{
			it = m_xList.erase(it);

		}
	}

	for (auto it = m_yList.begin(); it != m_yList.end(); ++it)
	{
		if ((*it)->m_object == object)
		{
			it = m_yList.erase(it);
		}
	}

	for (auto it = m_zList.begin(); it != m_zList.end(); ++it)
	{
		if ((*it)->m_object == object)
		{
			it = m_zList.erase(it);
		}
	}
}

void Collision::SweepAndPrune::InsertionSort()
{
	//std::cout << "X: ";
	InsertionSort(m_xList);
	//std::cout << "\nY: ";
	InsertionSort(m_yList);
	//std::cout << "\nZ: ";
	InsertionSort(m_zList);
	//std::cout << std::endl;
}

std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::SweepAndPrune::TestCollision()
{
	std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> result;
	auto start = std::chrono::high_resolution_clock::now();
	std::copy(m_collisionPairs.begin(), m_collisionPairs.end(), std::back_inserter(result));
	auto end = std::chrono::high_resolution_clock::now();

	auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	m_lastFrameCriteria["Time Spent - Collisions"] = (float) timeSpent;
	m_lastFrameCriteria["Intersection Tests"] = (float) m_lastFrameComparisons;
	m_lastFrameComparisons = 0;
	return result;
}
