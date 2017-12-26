#include "DummyMethod.h"
#include "..\Rendering\SceneObject.h"

Collision::DummyMethod::DummyMethod(std::vector<SceneObject *> *allObjects)
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

void Collision::DummyMethod::ObjectMoved(Rendering::SceneObject *object)
{
}

void Collision::DummyMethod::ObjectAdded(Rendering::SceneObject *object)
{
}

void Collision::DummyMethod::ObjectRemoved(Rendering::SceneObject *object)
{
}

std::unordered_set<std::pair<SceneObject *, SceneObject *>> Collision::DummyMethod::_TestCollision()
{
	return std::unordered_set<std::pair<SceneObject *, SceneObject *>>();
}
