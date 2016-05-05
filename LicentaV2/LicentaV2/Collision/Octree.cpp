#include "Octree.h"
#include <algorithm>
#include "..\Rendering\Models\Model.h"

Collision::Octree::Octree(std::vector<Rendering::IPhysicsObject *> *allObjects, glm::vec3 worldMin, glm::vec3 worldMax)
{
	m_allObjects = allObjects;
	m_worldCenter = worldMin + (worldMax - worldMin) / 2.f;
	m_worldHalfW = (worldMax.x - worldMin.x) / 2.f;

	m_root = new DataStructures::OctreeNode();
	m_root->m_center = m_worldCenter;
	m_root->m_halfW = m_worldHalfW;
}

Collision::Octree::~Octree()
{
	delete m_root;
}

std::vector<Rendering::IPhysicsObject *> Collision::Octree::TestCollision(Rendering::IPhysicsObject *queriedObject)
{
	return std::vector<Rendering::IPhysicsObject *>();
}

void Collision::Octree::Update()
{
	auto start = std::chrono::high_resolution_clock::now();

	delete m_root;

	m_root = new DataStructures::OctreeNode();
	m_root->m_center = m_worldCenter;
	m_root->m_halfW = m_worldHalfW;

	for (auto obj : *m_allObjects)
	{
		InsertIntoTree(obj);
	}	

	auto end = std::chrono::high_resolution_clock::now();

	auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	m_lastFrameCriteria["Time Spent - Structure Update"] = (float)timeSpent;
}

void Collision::Octree::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	if (!GetShowDebug())
		return;

	DrawRecursive(m_root, projectionMatrix, viewMatrix);
}

std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::Octree::TestCollision()
{
	std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> result;

	m_lastFrameComparisons = 0;
	auto start = std::chrono::high_resolution_clock::now();
	TestCollisionsRecursive(m_root, result);
	auto end = std::chrono::high_resolution_clock::now();

	auto timeSpent = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	m_lastFrameCriteria["Time Spent - Collisions"] = (float)timeSpent;
	m_lastFrameCriteria["Intersection Tests"] = (float)m_lastFrameComparisons;

	return result;
}

void Collision::Octree::SetParams(int splitThreshold, int maximumDepth)
{
	m_splitThreshold = splitThreshold;
	m_maximumDepth = maximumDepth;
}

void Collision::Octree::InsertIntoTree(Rendering::IPhysicsObject *object)
{
	__InsertIntoTree(m_root, object);
}

void Collision::Octree::__InsertIntoTree(DataStructures::OctreeNode *node, Rendering::IPhysicsObject *object)
{
	int childIndex = 0;

	glm::vec3 dt = object->GetPosition() - node->m_center;
	childIndex = StraddleX(object, node) ? childIndex : dt.x > 0.f ? childIndex | (1 << 0) : childIndex;
	childIndex = StraddleY(object, node) ? childIndex : dt.y > 0.f ? childIndex | (1 << 1) : childIndex;
	childIndex = StraddleZ(object, node) ? childIndex : dt.z > 0.f ? childIndex | (1 << 2) : childIndex;
	
	float newHalfWidth = node->m_halfW / 2;
	glm::vec3 centerOffset;
	centerOffset.x = childIndex & (1 << 0) ? newHalfWidth : -newHalfWidth;
	centerOffset.y = childIndex & (1 << 1) ? newHalfWidth : -newHalfWidth;
	centerOffset.z = childIndex & (1 << 2) ? newHalfWidth : -newHalfWidth;
	glm::vec3 newCenter = node->m_center + centerOffset;
		
	if (CompletelyInside(object, newCenter, newHalfWidth))
	{
		if (node->m_children[childIndex] == NULL)
		{
			DataStructures::OctreeNode *toBeAdded = new DataStructures::OctreeNode();
			toBeAdded->m_halfW = newHalfWidth;
			toBeAdded->m_center = newCenter;
			node->m_children[childIndex] = toBeAdded;
		}

		__InsertIntoTree(node->m_children[childIndex], object);
	}
	else
	{
		node->m_objects.push_back(object);
	}
}

bool Collision::Octree::CompletelyInside(Rendering::IPhysicsObject *object, glm::vec3 center, float halfWidth)
{
	glm::vec3 nodeDimMax = center + halfWidth;
	glm::vec3 nodeDimMin = center - halfWidth;

// 	if (object->GetMaxCoords().x > nodeDimMax.x) return false;
// 	if (object->GetMinCoords().x < nodeDimMin.x) return false;
// 
// 	if (object->GetMaxCoords().y > nodeDimMax.y) return false;
// 	if (object->GetMinCoords().y < nodeDimMin.y) return false;
// 
// 	if (object->GetMaxCoords().z > nodeDimMax.z) return false;
// 	if (object->GetMinCoords().z < nodeDimMin.z) return false;

	return !(object->GetMaxCoords().x > nodeDimMax.x || object->GetMinCoords().x < nodeDimMin.x || object->GetMaxCoords().y > nodeDimMax.y ||
		object->GetMinCoords().y < nodeDimMin.y || object->GetMaxCoords().z > nodeDimMax.z || object->GetMinCoords().z < nodeDimMin.z);
	//return true;
}

bool Collision::Octree::StraddleX(Rendering::IPhysicsObject *object, DataStructures::OctreeNode *node)
{
	float nodeDimMax = node->m_center.x + node->m_halfW;
	float nodeDimMin = node->m_center.x - node->m_halfW;

	return object->GetMaxCoords().x > nodeDimMax || object->GetMinCoords().x < nodeDimMin;
}

bool Collision::Octree::StraddleY(Rendering::IPhysicsObject *object, DataStructures::OctreeNode *node)
{
	float nodeDimMax = node->m_center.y + node->m_halfW;
	float nodeDimMin = node->m_center.y - node->m_halfW;

	return object->GetMaxCoords().y > nodeDimMax || object->GetMinCoords().y < nodeDimMin;
}

bool Collision::Octree::StraddleZ(Rendering::IPhysicsObject *object, DataStructures::OctreeNode *node)
{
	float nodeDimMax = node->m_center.z + node->m_halfW;
	float nodeDimMin = node->m_center.z - node->m_halfW;

	return object->GetMaxCoords().z > nodeDimMax|| object->GetMinCoords().z < nodeDimMin;
}

void Collision::Octree::DrawRecursive(DataStructures::OctreeNode *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	if (!node)
		return;

	glm::mat4 modelMatrix = glm::translate(glm::mat4(1), node->m_center) * glm::scale(glm::mat4(1), glm::vec3(node->m_halfW * 2));
	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * modelMatrix;


	glUniform1i(2, node->m_objects.empty() ? 0 : BOUNDINGBOX);
	glUniformMatrix4fv(3, 1, false, &MVPMatrix[0][0]);
	glBindVertexArray(m_vao);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

	for (int i = 0; i < node->m_children.size(); ++i)
	{
		DrawRecursive(node->m_children[i], projectionMatrix, viewMatrix);
	}
}

void Collision::Octree::TestCollisionsRecursive(DataStructures::OctreeNode *node, std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> &collisions)
{
	//static std::vector<
	static std::vector<DataStructures::OctreeNode *> stack;
	static int depth = 0;

	stack.resize(m_maximumDepth);

	if (depth >= m_maximumDepth - 1)
		return;

	stack[depth++] = node;

	for (int i = 0; i < depth; ++i)
	{
		for (auto firstObj : stack[i]->m_objects)
		{
			for (auto secondObj : node->m_objects)
			{
				if (firstObj == secondObj)
					break;

				m_lastFrameComparisons++;
				if (((Rendering::Models::Model *) firstObj)->GetBoundingBox()->Collides(((Rendering::Models::Model *) secondObj)->GetBoundingBox()))
				{
					collisions.push_back(std::make_pair(firstObj, secondObj));
				}
			}
		}
	}

	for (int i = 0; i < 8; ++i)
	{
		if (node->m_children[i])
		{
			TestCollisionsRecursive(node->m_children[i], collisions);
		}
	}

	depth--;
}

void Collision::Octree::ObjectMoved(Rendering::IPhysicsObject *object)
{
}

void Collision::Octree::ObjectAdded(Rendering::IPhysicsObject *object)
{
}

void Collision::Octree::ObjectRemoved(Rendering::IPhysicsObject *object)
{
}
