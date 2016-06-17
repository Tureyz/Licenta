#include "BVH.h"
#include <algorithm>
#include <iterator>

using namespace Collision;

Collision::BVH::BVH(std::vector<IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
	m_memoryUsed = sizeof(Collision::BVH);
	CreateTree(&m_root, m_allObjects->data(), m_allObjects->size());
}

void Collision::BVH::_Update()
{
	delete m_root;
	m_memoryUsed = sizeof(Collision::BVH);

	CreateTree(&m_root, m_allObjects->data(), m_allObjects->size());
}

void Collision::BVH::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	if (!GetShowDebug())
		return;

	glLineWidth(2);
	DrawRecursive(m_root, projectionMatrix, viewMatrix);
	glLineWidth(1);
}

void Collision::BVH::CreateTree(DataStructures::BVHTree **node, IPhysicsObject ** objects, size_t numObjects)
{
	if (numObjects <= 0)
	{
		return;
	}

	DataStructures::BVHTree *newNode = new DataStructures::BVHTree();
	*node = newNode;

	newNode->m_boundingBox = new Collision::DataStructures::BoundingBox(objects, numObjects);
	newNode->m_boundingBox->Create();
	newNode->m_boundingBox->SetVisible(true);
	newNode->m_objects = &objects[0];
	newNode->m_numObjects = numObjects;

	m_memoryUsed += sizeof(*newNode);
	m_memoryUsed += sizeof(*newNode->m_boundingBox);

	if (numObjects <= 1)
	{
		newNode->m_type = DataStructures::BVHTree::LEAF;
	}
	else
	{
		newNode->m_type = DataStructures::BVHTree::NODE;
		size_t k = SplitObjects(newNode);

		CreateTree(&newNode->m_left, &objects[0], k);
		CreateTree(&newNode->m_right, &objects[k], numObjects - k);
	}
}

size_t Collision::BVH::SplitObjects(Collision::DataStructures::BVHTree *node)
{
	enum SplittingAxisType { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2};

	Collision::DataStructures::BoundingBox *bb = node->m_boundingBox;	
	glm::vec3 boxLengths(bb->m_maxX - bb->m_minX, bb->m_maxY - bb->m_minY, bb->m_maxZ - bb->m_minZ);

	// Choose splitting axis as longest side, and splitting point as spatial median
	SplittingAxisType spl = AXIS_X;
	float splittingPoint = bb->m_minX + boxLengths.x / 2;

	if (boxLengths.y >= boxLengths.z && boxLengths.y >= boxLengths.x)
	{
		spl = AXIS_Y;
		splittingPoint = bb->m_minY + boxLengths.y / 2;
	}
	else if (boxLengths.z >= boxLengths.x && boxLengths.z >= boxLengths.y)
	{
		spl = AXIS_Z;
		splittingPoint = bb->m_minZ + boxLengths.z / 2;
	}

	std::vector<IPhysicsObject *> leftSide, rightSide;

	if (spl == AXIS_X)
	{		
		for (int i = 0; i < node->m_numObjects; ++i)
		{
			if (node->m_objects[i]->GetPosition().x < splittingPoint)
			{
				leftSide.push_back(node->m_objects[i]);				
			}
			else
			{
				rightSide.push_back(node->m_objects[i]);
			}
		}
		
	}
	else if (spl == AXIS_Y)
	{
		for (int i = 0; i < node->m_numObjects; ++i)
		{
			if (node->m_objects[i]->GetPosition().y < splittingPoint)
			{
				leftSide.push_back(node->m_objects[i]);
			}
			else
			{
				rightSide.push_back(node->m_objects[i]);
			}
		}
	}
	else if (spl == AXIS_Z)
	{
		for (int i = 0; i < node->m_numObjects; ++i)
		{
			if (node->m_objects[i]->GetPosition().z < splittingPoint)
			{
				leftSide.push_back(node->m_objects[i]);
			}
			else
			{
				rightSide.push_back(node->m_objects[i]);
			}
		}
	}

	int i = 0;	

	// If all objects fall on one side, choose object median instead.
	if (leftSide.empty())
	{
		if (spl == AXIS_X)
		{
			std::sort(rightSide.begin(), rightSide.end(), [](IPhysicsObject *a, IPhysicsObject *b) { return a->GetPosition().x < b->GetPosition().x; });
		}
		else if (spl == AXIS_Y)
		{
			std::sort(rightSide.begin(), rightSide.end(), [](IPhysicsObject *a, IPhysicsObject *b) { return a->GetPosition().y < b->GetPosition().y; });
		}
		else
		{
			std::sort(rightSide.begin(), rightSide.end(), [](IPhysicsObject *a, IPhysicsObject *b) { return a->GetPosition().z < b->GetPosition().z; });
		}

		for (auto obj : rightSide)
		{
			node->m_objects[i++] = obj;
		}

		return node->m_numObjects / 2;
	}

	if (rightSide.empty())
	{
		if (spl == AXIS_X)
		{
			std::sort(leftSide.begin(), leftSide.end(), [](IPhysicsObject *a, IPhysicsObject *b) { return a->GetPosition().x < b->GetPosition().x; });
		}
		else if (spl == AXIS_Y)
		{
			std::sort(leftSide.begin(), leftSide.end(), [](IPhysicsObject *a, IPhysicsObject *b) { return a->GetPosition().y < b->GetPosition().y; });
		}
		else
		{
			std::sort(leftSide.begin(), leftSide.end(), [](IPhysicsObject *a, IPhysicsObject *b) { return a->GetPosition().z < b->GetPosition().z; });
		}

		for (auto obj : leftSide)
		{
			node->m_objects[i++] = obj;
		}

		return node->m_numObjects / 2;
	}

	for (auto obj : leftSide)
	{
		node->m_objects[i++] = obj;
	}
	for (auto obj : rightSide)
	{
		node->m_objects[i++] = obj;
	}

	return leftSide.size();
}

void Collision::BVH::DrawRecursive(DataStructures::BVHTree * node, const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	if (!node)
		return;

	node->m_boundingBox->Draw(projectionMatrix, viewMatrix);
	DrawRecursive(node->m_left, projectionMatrix, viewMatrix);
	DrawRecursive(node->m_right, projectionMatrix, viewMatrix);
}

// false = right, true = left
bool Collision::BVH::ChildrenSelectionRule(DataStructures::BVHTree * left, DataStructures::BVHTree * right)
{	
	DataStructures::BoundingBox *leftBoundingBox = left->m_boundingBox;
	DataStructures::BoundingBox *rightBoundingBox = right->m_boundingBox;

	float leftVolume = (leftBoundingBox->m_maxX - leftBoundingBox->m_minX) * (leftBoundingBox->m_maxY - leftBoundingBox->m_minY) * (leftBoundingBox->m_maxZ - leftBoundingBox->m_minZ);
	float rightVolume = (rightBoundingBox->m_maxX - rightBoundingBox->m_minX) * (rightBoundingBox->m_maxY - rightBoundingBox->m_minY) * (rightBoundingBox->m_maxZ - rightBoundingBox->m_minZ);

	return right->m_type == DataStructures::BVHTree::LEAF || (left->m_type != DataStructures::BVHTree::LEAF && (leftVolume >= rightVolume));
}

std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::BVH::QueryBVHPairs(DataStructures::BVHTree *first, DataStructures::BVHTree *second)
{
	std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> result;

	std::vector<std::pair<Collision::DataStructures::BVHTree *, Collision::DataStructures::BVHTree *>> stack;

	m_lastFrameTests = 0;
	while (1)
	{
		m_lastFrameTests++;
		if (first && second && first->m_boundingBox->Collides(second->m_boundingBox))
		{
			if (first->m_type == Collision::DataStructures::BVHTree::LEAF && second->m_type == Collision::DataStructures::BVHTree::LEAF && first != second)
			{
				result.insert(std::make_pair(first->m_objects[0], second->m_objects[0]));
			}
			else
			{
				if (ChildrenSelectionRule(first, second))
				{
					stack.push_back(std::make_pair(second, first->m_right));
					first = first->m_left;
					continue;
				}
				else
				{
					stack.push_back(std::make_pair(second->m_right, first));
					second = second->m_left;
					continue;
				}
			}
		}
		if (stack.empty()) { break; }

		first = stack.back().first;
		second = stack.back().second;

		stack.pop_back();
	}
	return result;
}

void Collision::BVH::ObjectMoved(Rendering::IPhysicsObject *object)
{
}

void Collision::BVH::ObjectAdded(Rendering::IPhysicsObject *object)
{
}

void Collision::BVH::ObjectRemoved(Rendering::IPhysicsObject *object)
{
}

std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> Collision::BVH::_TestCollision()
{	
	return QueryBVHPairs(m_root, m_root);
}
