#include "../Rendering/Models/Model.h"
#include "SpatialHashing.h"
#include <cmath>
#include <cstdlib>

Collision::SpatialHashing::SpatialHashing(std::vector<Rendering::IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
	m_cellSize = 5; //default

	for (auto obj : (*m_allObjects))
	{
		InsertObject(obj);
	}
}

Collision::SpatialHashing::~SpatialHashing()
{

}

void Collision::SpatialHashing::_Update()
{
	m_hashTable.clear();
	m_bucketIndices.clear();
	m_memoryUsed = 0;

	for (auto obj : (*m_allObjects))
	{
		InsertObject(obj);
	}
}

void Collision::SpatialHashing::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{

}

void Collision::SpatialHashing::InsertObject(Rendering::IPhysicsObject *obj)
{
	Rendering::Models::Model *castedObj = ((Rendering::Models::Model *)obj);
	Collision::DataStructures::BoundingBox *bb = castedObj->GetBoundingBox();

	InsertPoint(glm::vec3(bb->m_minX, bb->m_minY, bb->m_minZ), obj);
	InsertPoint(glm::vec3(bb->m_minX, bb->m_maxY, bb->m_minZ), obj);
	InsertPoint(glm::vec3(bb->m_minX, bb->m_minY, bb->m_maxZ), obj);
	InsertPoint(glm::vec3(bb->m_minX, bb->m_maxY, bb->m_maxZ), obj);
	InsertPoint(glm::vec3(bb->m_maxX, bb->m_minY, bb->m_minZ), obj);
	InsertPoint(glm::vec3(bb->m_maxX, bb->m_maxY, bb->m_minZ), obj);
	InsertPoint(glm::vec3(bb->m_maxX, bb->m_minY, bb->m_maxZ), obj);
	InsertPoint(glm::vec3(bb->m_maxX, bb->m_maxY, bb->m_maxZ), obj);
}

void Collision::SpatialHashing::InsertPoint(glm::vec3 point, Rendering::IPhysicsObject *obj)
{
	size_t hashKey = ObjectHash(point);

	// 	if (m_bucketIndices.count(hashKey))
	// 	{
	// 		m_hashTable[m_bucketIndices[hashKey]].insert(obj);
	// 	}
	// 	else
	// 	{
	// 		m_hashTable.push_back(std::unordered_set<Rendering::IPhysicsObject*>());
	// 		m_bucketIndices[hashKey] = m_hashTable.size() - 1;
	// 	}

	if (!m_bucketIndices.count(hashKey))
	{
		m_hashTable.push_back(std::unordered_set<Rendering::IPhysicsObject*>());
		m_bucketIndices[hashKey] = m_hashTable.size() - 1;
		m_memoryUsed += sizeof(std::unordered_set<Rendering::IPhysicsObject*>) + sizeof(size_t);
	}

	m_hashTable[m_bucketIndices[hashKey]].insert(obj);
	m_memoryUsed += sizeof(Rendering::IPhysicsObject *);
}

void Collision::SpatialHashing::RemoveObject(Rendering::IPhysicsObject *obj)
{

	for (auto set : m_hashTable)
	{
		if (!set.empty())
		{
			set.erase(obj);
		}
	}
}

void Collision::SpatialHashing::MoveObject(Rendering::IPhysicsObject *obj)
{
	for (auto bucket : m_hashTable)
	{
		bucket.erase(obj);
	}

	InsertObject(obj);
}

size_t Collision::SpatialHashing::ObjectHash(glm::vec3 el)
{
	size_t result = 0;
	glm::vec3 dividedPoint = glm::floor(el / m_cellSize);

	if (dividedPoint.x < 0)
	{
		result |= ((size_t)1 << 15);
	}

	if (dividedPoint.y < 0)
	{
		result |= ((size_t)1 << 31);
	}

	if (dividedPoint.z < 0)
	{
		result |= ((size_t)1 << 47);
	}

	result |= (size_t)std::abs(dividedPoint.x);
	result |= ((size_t)std::abs(dividedPoint.y) << 16);
	result |= ((size_t)std::abs(dividedPoint.z) << 32);

	return result;
}

void Collision::SpatialHashing::ObjectMoved(Rendering::IPhysicsObject *object)
{
	// 	RemoveObject(object);
	// 	InsertObject(object);

		//MoveObject(object);
}

void Collision::SpatialHashing::ObjectAdded(Rendering::IPhysicsObject *object)
{
	//InsertObject(object);
}

void Collision::SpatialHashing::ObjectRemoved(Rendering::IPhysicsObject *object)
{
	//RemoveObject(object);
}

std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::SpatialHashing::_TestCollision()
{
	std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> result;
	m_lastFrameTests = 0;

	for (auto bucket : m_hashTable)
	{
		auto beg = bucket.begin();
		auto end = bucket.end();

		for (auto it = beg; it != end; ++it)
		{
			auto firstObj = (*it);
			for (auto it2 = it; it2 != end; ++it2)
			{
				auto secondObj = (*it2);
				m_lastFrameTests++;
				//if (secondObj != firstObj && ((Rendering::Models::Model *)firstObj)->GetBoundingBox()->Collides(((Rendering::Models::Model *)secondObj)->GetBoundingBox()))
				if (secondObj != firstObj && firstObj->SphereTest(secondObj))
				{
					result.insert(std::make_pair(firstObj, secondObj));
				}
			}
		}
	}

	return result;
}
