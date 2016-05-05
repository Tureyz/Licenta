#include "DummyMethod.h"
#include "..\Rendering\IPhysicsObject.h"

Collision::DummyMethod::DummyMethod(std::vector<IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
}

Collision::DummyMethod::~DummyMethod()
{
}

std::vector<Rendering::IPhysicsObject *> Collision::DummyMethod::TestCollision(Rendering::IPhysicsObject *queriedObject)
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::DummyMethod::Update()
{
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

std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> Collision::DummyMethod::TestCollision()
{
	return std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>>();
}
