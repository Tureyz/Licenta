#include "NarrowBVH.h"
#include <algorithm>
#include <iterator>

Collision::NarrowBVH::NarrowBVH(std::vector<Collision::DataStructures::CollisionTriangle *> *triangles)
{
	m_allObjects = triangles;
}

Collision::NarrowBVH::~NarrowBVH()
{

}

std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> Collision::NarrowBVH::TestCollision(NarrowBVH *other)
{
	return QueryBVHPairs(m_root, other->m_root);
}

void Collision::NarrowBVH::Update()
{
	// 	if (m_root)
	// 	{
	// 		delete m_root;
	// 	}
	// 
	// 	CreateTree(&m_root, m_allObjects->data(), m_allObjects->size());

	if (!m_root)
	{
		CreateTree(&m_root, m_allObjects->data(), m_allObjects->size());
	}
	else
	{
		UpdateBoxes(m_root);
	}

	//PrintIDsRecursive(m_root);

}

void Collision::NarrowBVH::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	if (!GetShowDebug() || !m_root)
		return;

	glLineWidth(2);
	DrawRecursive(m_root, projectionMatrix, viewMatrix);
	glLineWidth(1);
}

void Collision::NarrowBVH::CreateTree(DataStructures::BVHTree<DataStructures::CollisionTriangle *> **node, DataStructures::CollisionTriangle ** objects, size_t numObjects)
{
	if (numObjects <= 0)
	{
		return;
	}

	DataStructures::BVHTree<DataStructures::CollisionTriangle *> *newNode = new DataStructures::BVHTree<DataStructures::CollisionTriangle *>();
	*node = newNode;

	// 	newNode->m_boundingBox = new Collision::DataStructures::BoundingBox(objects, numObjects);
	// 	newNode->m_boundingBox->Create(TODO, TODO);
	// 	newNode->m_boundingBox->SetVisible(true);
	newNode->m_objects = &objects[0];
	newNode->m_numObjects = numObjects;
	//newNode->m_boundingBox.CreateVisualBody(GetModelManager()->CreateBasicVisualBody(Simulation::PhysicsObjectType::OBJ_LINE_CUBE));

	std::vector<std::pair<glm::vec3, glm::vec3>> bounds;
	for (int i = 0; i < numObjects; ++i)
	{
		auto tri = objects[i];
		glm::vec3 crtMin = tri->m_verts[0]->m_position, crtMax = tri->m_verts[0]->m_position;

		for (int j = 1; j < tri->m_verts.size(); ++j)
		{
			if (tri->m_verts[j]->m_position.x < crtMin.x)
				crtMin.x = tri->m_verts[j]->m_position.x;
			if (tri->m_verts[j]->m_position.x > crtMax.x)
				crtMax.x = tri->m_verts[j]->m_position.x;

			if (tri->m_verts[j]->m_position.y < crtMin.y)
				crtMin.y = tri->m_verts[j]->m_position.y;
			if (tri->m_verts[j]->m_position.y > crtMax.y)
				crtMax.y = tri->m_verts[j]->m_position.y;

			if (tri->m_verts[j]->m_position.z < crtMin.z)
				crtMin.z = tri->m_verts[j]->m_position.z;
			if (tri->m_verts[j]->m_position.z > crtMax.z)
				crtMax.z = tri->m_verts[j]->m_position.z;
		}


		bounds.push_back(std::make_pair(crtMin, crtMax));
	}

	newNode->m_boundingBox.UpdateValues(bounds);


	if (numObjects <= 1)
	{
		newNode->m_type = DataStructures::BVHTree<DataStructures::CollisionTriangle *>::LEAF;
	}
	else
	{
		newNode->m_type = DataStructures::BVHTree<DataStructures::CollisionTriangle *>::NODE;
		size_t k = SplitObjects(newNode);

		CreateTree(&newNode->m_left, &objects[0], k);
		CreateTree(&newNode->m_right, &objects[k], numObjects - k);
	}
}

void Collision::NarrowBVH::ObjectMoved(Collision::DataStructures::CollisionTriangle *object)
{

}

void Collision::NarrowBVH::ObjectAdded(Collision::DataStructures::CollisionTriangle *object)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::NarrowBVH::ObjectRemoved(Collision::DataStructures::CollisionTriangle *object)
{
	throw std::logic_error("The method or operation is not implemented.");
}

size_t Collision::NarrowBVH::SplitObjects(Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node)
{
	enum SplittingAxisType { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };

	Collision::DataStructures::BoundingBox bb = node->m_boundingBox;
	glm::vec3 boxLengths(node->m_boundingBox.m_maxX - node->m_boundingBox.m_minX, node->m_boundingBox.m_maxY - node->m_boundingBox.m_minY, node->m_boundingBox.m_maxZ - node->m_boundingBox.m_minZ);

	// Choose splitting axis as longest side, and splitting point as spatial median
	SplittingAxisType spl = AXIS_X;
	float splittingPoint = node->m_boundingBox.m_minX + boxLengths.x / 2;

	if (boxLengths.y >= boxLengths.z && boxLengths.y >= boxLengths.x)
	{
		spl = AXIS_Y;
		splittingPoint = node->m_boundingBox.m_minY + boxLengths.y / 2;
	}
	else if (boxLengths.z >= boxLengths.x && boxLengths.z >= boxLengths.y)
	{
		spl = AXIS_Z;
		splittingPoint = node->m_boundingBox.m_minZ + boxLengths.z / 2;
	}

	std::vector<DataStructures::CollisionTriangle *> leftSide, rightSide;

	if (spl == AXIS_X)
	{
		for (int i = 0; i < node->m_numObjects; ++i)
		{
			if (node->m_objects[i]->GetCenter().x < splittingPoint)
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
			if (node->m_objects[i]->GetCenter().y < splittingPoint)
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
			if (node->m_objects[i]->GetCenter().z < splittingPoint)
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
			std::sort(rightSide.begin(), rightSide.end(), [](DataStructures::CollisionTriangle *a, DataStructures::CollisionTriangle *b) { return a->GetCenter().x < b->GetCenter().x; });
		}
		else if (spl == AXIS_Y)
		{
			std::sort(rightSide.begin(), rightSide.end(), [](DataStructures::CollisionTriangle *a, DataStructures::CollisionTriangle *b) { return a->GetCenter().y < b->GetCenter().y; });
		}
		else
		{
			std::sort(rightSide.begin(), rightSide.end(), [](DataStructures::CollisionTriangle *a, DataStructures::CollisionTriangle *b) { return a->GetCenter().z < b->GetCenter().z; });
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
			std::sort(leftSide.begin(), leftSide.end(), [](DataStructures::CollisionTriangle *a, DataStructures::CollisionTriangle *b) { return a->GetCenter().x < b->GetCenter().x; });
		}
		else if (spl == AXIS_Y)
		{
			std::sort(leftSide.begin(), leftSide.end(), [](DataStructures::CollisionTriangle *a, DataStructures::CollisionTriangle *b) { return a->GetCenter().y < b->GetCenter().y; });
		}
		else
		{
			std::sort(leftSide.begin(), leftSide.end(), [](DataStructures::CollisionTriangle *a, DataStructures::CollisionTriangle *b) { return a->GetCenter().z < b->GetCenter().z; });
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

void Collision::NarrowBVH::DrawRecursive(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{

}

bool Collision::NarrowBVH::ChildrenSelectionRule(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *left, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *right)
{
	float leftVolume = (left->m_boundingBox.m_maxX - left->m_boundingBox.m_minX) * (left->m_boundingBox.m_maxY - left->m_boundingBox.m_minY) * (left->m_boundingBox.m_maxZ - left->m_boundingBox.m_minZ);
	float rightVolume = (right->m_boundingBox.m_maxX - right->m_boundingBox.m_minX) * (right->m_boundingBox.m_maxY - right->m_boundingBox.m_minY) * (right->m_boundingBox.m_maxZ - right->m_boundingBox.m_minZ);

	return right->IsLeaf() || (!left->IsLeaf() && (leftVolume >= rightVolume));
}

std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> Collision::NarrowBVH::QueryBVHPairs(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *first, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *second)
{
	std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> result;

	QueryBVHPairsRecursive(first, second, result);

	return result;
}

std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> Collision::NarrowBVH::QueryBVHPairsLoop(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *first, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *second)
{
	std::unordered_set<std::pair<DataStructures::CollisionTriangle *, DataStructures::CollisionTriangle *>> result;

	std::vector<std::pair<Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *> *, Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *> *>> stack;

	while (1)
	{
		// 		if (first->m_type == Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *>::LEAF)
		// 		{
		// 			std::cout << "L1" << std::endl;
		// 		}
		// 		if (second->m_type == Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *>::LEAF)
		// 		{
		// 			std::cout << "L2" << std::endl;
		// 		}
		if (first && second && first->m_boundingBox.Collides(second->m_boundingBox))
		{


			//if (first->m_type == Collision::DataStructures::BVHTree::LEAF && second->m_type == Collision::DataStructures::BVHTree::LEAF && first != second)
			if (first->m_type == Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *>::LEAF && second->m_type == Collision::DataStructures::BVHTree<DataStructures::CollisionTriangle *>::LEAF && first != second)
			{
				//if (first->m_objects[0]->SphereTest(second->m_objects[0]))
				if (Collision::DataStructures::CollisionTriangle::TriangleTest(*first->m_objects[0], *second->m_objects[0]))
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
		if (stack.empty())
		{
			break;
		}

		first = stack.back().first;
		second = stack.back().second;

		stack.pop_back();
	}

	return result;
}

void Collision::NarrowBVH::QueryBVHPairsRecursive(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *first, DataStructures::BVHTree<DataStructures::CollisionTriangle *> *second, std::unordered_set<std::pair<DataStructures::CollisionTriangle *, DataStructures::CollisionTriangle *>> &result, int indent /*= 0*/)
{
	if (!first || !second)
		return;

	if (!first->m_boundingBox.Collides(second->m_boundingBox))
		return;

	if (first->IsLeaf() && second->IsLeaf())
	{
		if (Collision::DataStructures::CollisionTriangle::TriangleTest(*first->m_objects[0], *second->m_objects[0]))
			result.insert(std::make_pair(first->m_objects[0], second->m_objects[0]));
	}
	else
	{
		if (ChildrenSelectionRule(first, second))
		{
			QueryBVHPairsRecursive(first->m_left, second, result, indent + 1);
			QueryBVHPairsRecursive(first->m_right, second, result, indent + 1);
		}
		else
		{
			QueryBVHPairsRecursive(first, second->m_left, result, indent + 1);
			QueryBVHPairsRecursive(first, second->m_right, result, indent + 1);
		}
	}
}

void Collision::NarrowBVH::PrintIDsRecursive(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node)
{
	if (!node)
	{
		std::cout << "|";
		return;
	}
	std::cout << node->m_type;

	PrintIDsRecursive(node->m_left);
	PrintIDsRecursive(node->m_right);
}

void Collision::NarrowBVH::UpdateBoxes(DataStructures::BVHTree<DataStructures::CollisionTriangle *> *node)
{
	if (!node)
		return;

	UpdateBoxes(node->m_left);
	UpdateBoxes(node->m_right);

	glm::vec3 minCoords, maxCoords;

	if (node->IsLeaf())
	{
		auto tri = node->m_objects[0];

		minCoords = tri->m_verts[0]->m_position;
		maxCoords = tri->m_verts[0]->m_position;

		for (int i = 1; i < 3; ++i)
		{
			if (tri->m_verts[i]->m_position.x < minCoords.x) minCoords.x = tri->m_verts[i]->m_position.x;
			if (tri->m_verts[i]->m_position.y < minCoords.y) minCoords.y = tri->m_verts[i]->m_position.y;
			if (tri->m_verts[i]->m_position.z < minCoords.z) minCoords.z = tri->m_verts[i]->m_position.z;

			if (tri->m_verts[i]->m_position.x > maxCoords.x) maxCoords.x = tri->m_verts[i]->m_position.x;
			if (tri->m_verts[i]->m_position.y > maxCoords.y) maxCoords.y = tri->m_verts[i]->m_position.y;
			if (tri->m_verts[i]->m_position.z > maxCoords.z) maxCoords.z = tri->m_verts[i]->m_position.z;
		}

	}
	else
	{
		auto bbl = node->m_left->m_boundingBox;
		auto bbr = node->m_right->m_boundingBox;

		minCoords.x = bbl.m_minX < bbr.m_minX ? bbl.m_minX : bbr.m_minX;
		minCoords.y = bbl.m_minY < bbr.m_minY ? bbl.m_minY : bbr.m_minY;
		minCoords.z = bbl.m_minZ < bbr.m_minZ ? bbl.m_minZ : bbr.m_minZ;

		maxCoords.x = bbl.m_maxX > bbr.m_maxX ? bbl.m_maxX : bbr.m_maxX;
		maxCoords.y = bbl.m_maxY > bbr.m_maxY ? bbl.m_maxY : bbr.m_maxY;
		maxCoords.z = bbl.m_maxZ > bbr.m_maxZ ? bbl.m_maxZ : bbr.m_maxZ;

	}

	node->m_boundingBox.UpdateValues(minCoords, maxCoords);
}
