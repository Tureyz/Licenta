#include "SphereToSphereTest.h"

Collision::SphereToSphereTest::SphereToSphereTest(std::vector<Rendering::SceneObject *> *allObjects)
{
	m_allObjects = allObjects;
}

void Collision::SphereToSphereTest::DrawDebug(const glm::mat4& viewProjection)
{
}

void Collision::SphereToSphereTest::ObjectMoved(Rendering::SceneObject *object)
{

}

void Collision::SphereToSphereTest::ObjectAdded(Rendering::SceneObject *object)
{

}

void Collision::SphereToSphereTest::ObjectRemoved(Rendering::SceneObject *object)
{

}

std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> Collision::SphereToSphereTest::_TestCollision()
{
	std::unordered_set<std::pair<SceneObject *, SceneObject *>> result;

	for (auto firstObj : *m_allObjects)
	{
		for (auto secondObj : *m_allObjects)
		{
// 			if (firstObj != secondObj)
// 			{
// 				Rendering::Models::Sphere *firstSphere = (Rendering::Models::Sphere *) firstObj;
// 				Rendering::Models::Sphere *secondSphere = (Rendering::Models::Sphere *) secondObj;
// 
// 				auto firstCenter = firstSphere->GetPosition();
// 				auto secondCenter = secondSphere->GetPosition();
// 
// 				//was initial verts
// 				auto firstRadius = glm::distance(firstCenter, firstSphere->GetVisualBody().m_verts[0].m_position);
// 				auto secondRadius = glm::distance(secondCenter, secondSphere->GetVisualBody().m_verts[0].m_position);
// 
// 				if (glm::distance(firstCenter, secondCenter) < firstRadius + secondRadius)
// 				{
// 					result.insert(std::make_pair(firstObj, secondObj));
// 				}
// 			}
		}
	}

	return result;
}

void Collision::SphereToSphereTest::_Update()
{

}
