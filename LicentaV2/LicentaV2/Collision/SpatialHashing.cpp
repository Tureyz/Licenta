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

std::vector<Rendering::IPhysicsObject *> Collision::SpatialHashing::TestCollision(Rendering::IPhysicsObject *queriedObject)
{
	return std::vector<Rendering::IPhysicsObject *>();
}

void Collision::SpatialHashing::Update()
{
	auto start = std::chrono::high_resolution_clock::now();

	m_hashTable.clear();
	m_bucketIndices.clear();

	for (auto obj : (*m_allObjects))
	{
		InsertObject(obj);
	}

	auto end = std::chrono::high_resolution_clock::now();

	auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	m_lastFrameCriteria["Time Spent - Structure Update"] = (float)timeSpent;
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
	}

	m_hashTable[m_bucketIndices[hashKey]].insert(obj);
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

std::string Collision::SpatialHashing::BinaryToString(short int x)
{
	return "(" + std::to_string(x) + ") " + std::bitset<sizeof(x) * 8>(x).to_string();
}

std::string Collision::SpatialHashing::BinaryToString(size_t x)
{
	auto result = std::bitset<sizeof(x) * 8>(x).to_string();

	result.insert(16, " ");
	result.insert(33, " ");
	result.insert(50, " ");
	return "(" + std::to_string(x) + ") " + result;	
}

std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::SpatialHashing::TestCollision()
{
 	std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> result;
	m_lastFrameComparisons = 0;
	auto start = std::chrono::high_resolution_clock::now();
	
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
				m_lastFrameComparisons++;
				if (secondObj != firstObj && ((Rendering::Models::Model *)firstObj)->GetBoundingBox()->Collides(((Rendering::Models::Model *)secondObj)->GetBoundingBox()))
				{
					result.push_back(std::make_pair(firstObj, secondObj));
				}
			}
		}
	}

	auto end = std::chrono::high_resolution_clock::now();

	auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	m_lastFrameCriteria["Time Spent - Collisions"] = (float)timeSpent;
	m_lastFrameCriteria["Intersection Tests"] = (float)m_lastFrameComparisons;

	return result;
}