#include "SweepAndPrune.h"
#include "../Rendering/Models/Model.h"
#include <iterator>

Collision::SweepAndPrune::SweepAndPrune(std::vector<Rendering::IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;

	for (auto obj : *m_allObjects)
	{
		Rendering::Models::Model *castedObj = (Rendering::Models::Model *)obj;

		Projection xProjBegin;
		xProjBegin.m_coord = castedObj->GetBoundingBox()->m_minX;
		xProjBegin.m_begin = true;
		xProjBegin.m_object = obj;

		Projection xProjEnd;
		xProjEnd.m_coord = castedObj->GetBoundingBox()->m_maxX;
		xProjEnd.m_begin = false;
		xProjEnd.m_object = obj;

		//m_xList.InsertBack(xProjBegin);
		//m_xList.InsertBack(xProjEnd);
		m_xList.push_back(xProjBegin);
		m_xList.push_back(xProjEnd);

		Projection yProjBegin;
		yProjBegin.m_coord = castedObj->GetBoundingBox()->m_minY;
		yProjBegin.m_begin = true;
		yProjBegin.m_object = obj;

		Projection yProjEnd;
		yProjEnd.m_coord = castedObj->GetBoundingBox()->m_maxY;
		yProjEnd.m_begin = false;
		yProjEnd.m_object = obj;

// 		m_yList.InsertBack(yProjBegin);
// 		m_yList.InsertBack(yProjEnd);
		m_yList.push_back(yProjBegin);
		m_yList.push_back(yProjEnd);

		Projection zProjBegin;
		zProjBegin.m_coord = castedObj->GetBoundingBox()->m_minZ;
		zProjBegin.m_begin = true;
		zProjBegin.m_object = obj;

		Projection zProjEnd;
		zProjEnd.m_coord = castedObj->GetBoundingBox()->m_maxZ;
		zProjEnd.m_begin = false;
		zProjEnd.m_object = obj;

// 		m_zList.InsertBack(zProjBegin);
// 		m_zList.InsertBack(zProjEnd);
		m_zList.push_back(zProjBegin);
		m_zList.push_back(zProjEnd);
	}
}

Collision::SweepAndPrune::~SweepAndPrune()
{

}

std::vector<Rendering::IPhysicsObject *> Collision::SweepAndPrune::TestCollision(Rendering::IPhysicsObject *queriedObject)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::SweepAndPrune::Update()
{
	InsertionSort();
}

void Collision::SweepAndPrune::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
}

void Collision::SweepAndPrune::InsertionSort(std::vector<Projection> &list)
{
// 	for (DataStructures::ListNode<Projection> *firstCrawler = list.m_head->m_next; firstCrawler != list.m_tail, firstCrawler != NULL; firstCrawler = firstCrawler->m_next)
// 	{		
// 		DataStructures::ListNode<Projection> *secondCrawler = firstCrawler->m_prev;
// 
// 		while (secondCrawler && secondCrawler->m_data.m_coord > firstCrawler->m_data.m_coord)
// 		{
// 			if (firstCrawler->m_data.m_begin && !secondCrawler->m_data.m_begin)
// 			{
// 				if (((Rendering::Models::Model *)firstCrawler->m_data.m_object)->GetBoundingBox()->Collides(((Rendering::Models::Model *)secondCrawler->m_data.m_object)->GetBoundingBox()))
// 				{
// 					std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> fst = std::make_pair(firstCrawler->m_data.m_object, secondCrawler->m_data.m_object);
// 					std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> snd = std::make_pair(secondCrawler->m_data.m_object, firstCrawler->m_data.m_object);
// 
// 					m_collisionPairs.push_back(fst);
// 					m_collisionPairs.push_back(snd);
// 				}
// 			}
// 
// 			if (!firstCrawler->m_data.m_begin && secondCrawler->m_data.m_begin)
// 			{
// 				std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> fst = std::make_pair(firstCrawler->m_data.m_object, secondCrawler->m_data.m_object);
// 				std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> snd = std::make_pair(secondCrawler->m_data.m_object, firstCrawler->m_data.m_object);
// 
// 				auto it = std::find(m_collisionPairs.begin(), m_collisionPairs.end(), fst);
// 				if (it != m_collisionPairs.end())
// 					m_collisionPairs.erase(it);
// 
// 				it = std::find(m_collisionPairs.begin(), m_collisionPairs.end(), snd);
// 				if (it != m_collisionPairs.end())
// 					m_collisionPairs.erase(it);
// 			}
// 
// 			if (secondCrawler->m_next)
// 			{
// 				secondCrawler->m_next->m_data = secondCrawler->m_data;
// 				secondCrawler = secondCrawler->m_prev;
// 			}
// 		}
// 		if (secondCrawler->m_next)
// 		{
// 			secondCrawler->m_next->m_data = firstCrawler->m_data;
// 		}
// 	}

	for (int i = 1; i < list.size(); ++i)
	{
		Projection firstProj = list[i];
		float coord = firstProj.m_coord;

		int j = i - 1;

		while (j >= 0 && list[j].m_coord > coord)
		{
			//std::cout << "SWAP\n";
			Projection secondProj = list[j];

			if (firstProj.m_begin && !secondProj.m_begin)
			{
				if (((Rendering::Models::Model *)firstProj.m_object)->GetBoundingBox()->Collides(((Rendering::Models::Model *)secondProj.m_object)->GetBoundingBox()))
				{
					//std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> fst = std::make_pair(firstProj.m_object, secondProj.m_object);
					//std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> snd = std::make_pair(secondProj.m_object, firstProj.m_object);

					m_collisionPairs.insert(std::make_pair(firstProj.m_object, secondProj.m_object));
					//m_collisionPairs.insert(snd);
				}
			}

			if (!firstProj.m_begin && secondProj.m_begin)
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

				m_collisionPairs.erase(std::make_pair(firstProj.m_object, secondProj.m_object));
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

	std::copy(m_collisionPairs.begin(), m_collisionPairs.end(), std::back_inserter(result));
	return result;
}
