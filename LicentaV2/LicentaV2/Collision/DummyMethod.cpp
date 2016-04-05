#include "DummyMethod.h"
#include "..\Rendering\IPhysicsObject.h"

Collision::DummyMethod::DummyMethod(std::vector<IPhysicsObject *> *allObjects)
{
	m_allObjects = allObjects;
}

Collision::DummyMethod::~DummyMethod()
{
}

std::vector<IPhysicsObject *> Collision::DummyMethod::TestCollision(IPhysicsObject *queriedObject)
{
	std::vector< IPhysicsObject *> result;
	for (int i = 1; i < glm::min<size_t>(11, m_allObjects->size()); ++i)
	{
		result.push_back((*m_allObjects)[i]);
	}

	return result;
}

void Collision::DummyMethod::Update()
{
	throw std::logic_error("The method or operation is not implemented.");
}

void Collision::DummyMethod::DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix)
{
	throw std::logic_error("The method or operation is not implemented.");
}

std::vector<std::pair<IPhysicsObject *, IPhysicsObject *>> Collision::DummyMethod::TestCollision()
{
	throw std::logic_error("The method or operation is not implemented.");
}
