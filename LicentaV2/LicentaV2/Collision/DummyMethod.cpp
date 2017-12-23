#include "DummyMethod.h"
#include "..\Rendering\IPhysicsObject.h"

Collision::DummyMethod::DummyMethod(std::vector<IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
	m_memoryUsed = 0;
}

Collision::DummyMethod::~DummyMethod()
{
}

void Collision::DummyMethod::_Update()
{
}

void Collision::DummyMethod::DrawDebug(const glm::mat4& viewProjection)
{
}

void Collision::DummyMethod::ObjectMoved(Rendering::IPhysicsObject *object)
{
}

void Collision::DummyMethod::ObjectAdded(Rendering::IPhysicsObject *object)
{
}

void Collision::DummyMethod::ObjectRemoved(Rendering::IPhysicsObject *object)
{
}

std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> Collision::DummyMethod::_TestCollision()
{
	return std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>>();
}
