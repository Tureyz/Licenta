#include "DummyMethod.h"
#include "..\Rendering\IPhysicsObject.h"

Collision::DummyMethod::DummyMethod(std::vector<IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
}

Collision::DummyMethod::~DummyMethod()
{
}

void Collision::DummyMethod::_Update()
{
	m_memoryCounter.resetAll();
	m_memoryCounter.addDynamic(0);
}

void Collision::DummyMethod::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
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
