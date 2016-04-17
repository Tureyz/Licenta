#include "KDTree.h"

Collision::KDTree::KDTree(std::vector<Rendering::IPhysicsObject *> *allObjects, glm::vec3 worldMin, glm::vec3 worldMax)
{
	m_allObjects = allObjects;
	m_worldMin = worldMin;
	m_worldMax = worldMax;

	m_root = new DataStructures::KDTreeNode();
	m_root->m_axis = DataStructures::AXIS_X;
	m_root->m_splittingPoint = m_worldMin.x + (m_worldMax.x - m_worldMin.x) / 2;
	m_root->m_minCoords = worldMin;
	m_root->m_maxCoords = worldMax;
}

Collision::KDTree::~KDTree()
{
	delete m_root;
}

std::vector<Rendering::IPhysicsObject *> Collision::KDTree::TestCollision(Rendering::IPhysicsObject *queriedObject)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::KDTree::Update()
{
	delete m_root;

	m_root = new DataStructures::KDTreeNode();
	m_root->m_axis = DataStructures::AXIS_X;
	m_root->m_splittingPoint = m_worldMin.x + (m_worldMax.x - m_worldMin.x) / 2;
	m_root->m_minCoords = m_worldMin;
	m_root->m_maxCoords = m_worldMax;

	for (auto obj : *m_allObjects)
	{
		InsertIntoTree(obj);
	}
}

void Collision::KDTree::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	if (!GetShowDebug())
		return;

	DrawRecursive(m_root, projectionMatrix, viewMatrix);
}

void Collision::KDTree::InsertIntoTree(Rendering::IPhysicsObject *object)
{
	__InsertIntoTree(m_root, object);
}

void Collision::KDTree::__InsertIntoTree(DataStructures::KDTreeNode *node, Rendering::IPhysicsObject *object)
{
	float dt = node->m_axis == (DataStructures::AXIS_X ? object->GetPosition().x : node->m_axis == DataStructures::AXIS_Y ? object->GetPosition().y : object->GetPosition().z) - node->m_splittingPoint;
	int childIndex = Straddle(object, node) ? 0 : dt > 0.f ? 1 : 0;
	
}

void Collision::KDTree::DrawRecursive(DataStructures::KDTreeNode *node, const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{

}

bool Collision::KDTree::StraddleX(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node)
{
	return object->GetMaxCoords().x > node->m_maxCoords.x || object->GetMinCoords().x < node->m_minCoords.x;
}

bool Collision::KDTree::StraddleY(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node)
{
	return object->GetMaxCoords().y > node->m_maxCoords.y || object->GetMinCoords().y < node->m_minCoords.y;
}

bool Collision::KDTree::StraddleZ(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node)
{
	return object->GetMaxCoords().z > node->m_maxCoords.z || object->GetMinCoords().z < node->m_minCoords.z;
}

bool Collision::KDTree::Straddle(Rendering::IPhysicsObject *object, DataStructures::KDTreeNode *node)
{
	switch (node->m_axis)
	{
	case Collision::DataStructures::AXIS_X:
		return StraddleX(object, node);
	case Collision::DataStructures::AXIS_Y:
		return StraddleY(object, node);
	default:
		return StraddleZ(object, node);
	}
}

std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> Collision::KDTree::TestCollision()
{
	throw std::logic_error("The method or operation is not implemented.");
}
