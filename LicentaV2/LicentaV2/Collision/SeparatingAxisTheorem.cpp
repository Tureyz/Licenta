#include "SeparatingAxisTheorem.h"
#include "../Dependencies/glm/gtx/projection.hpp"


Collision::SeparatingAxisTheorem::SeparatingAxisTheorem(std::vector<Rendering::SceneObject *> *allObjects)
{
	m_allObjects = allObjects;
}

Collision::SeparatingAxisTheorem::~SeparatingAxisTheorem()
{
}

void Collision::SeparatingAxisTheorem::_Update()
{
}

void Collision::SeparatingAxisTheorem::DrawDebug(const glm::mat4& viewProjection)
{
}

void Collision::SeparatingAxisTheorem::ObjectMoved(Rendering::SceneObject *object)
{
}

void Collision::SeparatingAxisTheorem::ObjectAdded(Rendering::SceneObject *object)
{
}

void Collision::SeparatingAxisTheorem::ObjectRemoved(Rendering::SceneObject *object)
{
}

std::vector<glm::vec3> Collision::SeparatingAxisTheorem::GetTestingAxes(Rendering::SceneObject *obj1, Rendering::SceneObject *obj2)
{
	const std::vector<VertexFormat> verts1 = obj1->GetVertices();
	const std::vector<unsigned int> indices1 = obj1->GetIndices();
	const std::vector<VertexFormat> verts2 = obj2->GetVertices();
	const std::vector<unsigned int> indices2 = obj2->GetIndices();

	std::vector<glm::vec3> result;
	for (int i = 0; i < indices1.size() - 3; i += 3)
	{
		glm::vec3 v11 = verts1[indices1[i]].m_position;
		glm::vec3 v12 = verts1[indices1[i + 1]].m_position;
		glm::vec3 v13 = verts1[indices1[i + 2]].m_position;

		glm::vec3 edge11 = v12 - v11;
		glm::vec3 edge12 = v13 - v11;
		glm::vec3 edge13 = v13 - v12;

		glm::vec3 surfaceNormal1 = glm::normalize(glm::cross(edge11, edge12));

		result.push_back(surfaceNormal1);

		for (int j = 0; j < indices2.size() - 3; j += 3)
		{
			glm::vec3 v21 = verts2[indices2[j]].m_position;
			glm::vec3 v22 = verts2[indices2[j + 1]].m_position;
			glm::vec3 v23 = verts2[indices2[j + 2]].m_position;

			glm::vec3 edge21 = v22 - v21;
			glm::vec3 edge22 = v23 - v21;
			glm::vec3 edge23 = v23 - v22;

			glm::vec3 surfaceNormal2 = glm::normalize(glm::cross(edge21, edge22));

			result.push_back(surfaceNormal2);

			result.push_back(glm::normalize(glm::cross(edge11, edge21)));
			result.push_back(glm::normalize(glm::cross(edge11, edge22)));
			result.push_back(glm::normalize(glm::cross(edge11, edge23)));

			result.push_back(glm::normalize(glm::cross(edge12, edge21)));
			result.push_back(glm::normalize(glm::cross(edge12, edge22)));
			result.push_back(glm::normalize(glm::cross(edge12, edge23)));

			result.push_back(glm::normalize(glm::cross(edge13, edge21)));
			result.push_back(glm::normalize(glm::cross(edge13, edge22)));
			result.push_back(glm::normalize(glm::cross(edge13, edge23)));

		}
	}

	return result;
}

std::pair<float, float> Collision::SeparatingAxisTheorem::GetProjection(Rendering::SceneObject *obj, const glm::vec3 &axis)
{
	std::pair<float, float> result((float) INT_MAX, (float) INT_MIN);

	std::vector<VertexFormat> verts = obj->GetVertices();

	for (int i = 1; i < verts.size(); ++i)
	{
		float proj = glm::dot(axis, verts[i].m_position);

		if (proj < result.first)
		{
			result.first = proj;
		}
		if (proj > result.second)
		{
			result.second = proj;
		}
	}

	return result;
}

bool Collision::SeparatingAxisTheorem::TestTwoObjects(Rendering::SceneObject *obj1, Rendering::SceneObject *obj2)
{
	std::vector<glm::vec3> axes = GetTestingAxes(obj1, obj2);
//	std::vector<glm::vec3> axes2 = GetTestingAxes(obj2, TODO);

	//axes.insert(axes.end(), axes2.begin(), axes2.end());

	for (auto axis : axes)
	{
		if (!TestProjectionOverlap(GetProjection(obj1, axis), GetProjection(obj2, axis)))
		{
			return false;
		}
	}

// 	for (auto axis : axes2)
// 	{
// 		if (!TestProjectionOverlap(GetProjection(obj1, axis), GetProjection(obj2, axis)))
// 		{
// 			return false;
// 		}
// 	}
	return true;
}

bool Collision::SeparatingAxisTheorem::TestProjectionOverlap(const std::pair<float, float> &proj1, const std::pair<float, float> &proj2)
{
	return (proj2.first >= proj1.first && proj2.first < proj1.second) || (proj1.first >= proj2.first && proj1.first < proj2.second);
}


std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> Collision::SeparatingAxisTheorem::_TestCollision()
{	
	std::unordered_set<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>> result;

	for (int i = 0; i < m_allObjects->size(); ++i)
	{
		auto obj1 = (*m_allObjects)[i];
		for (int j = i + 1; j < m_allObjects->size(); ++j)
		{
			auto obj2 = (*m_allObjects)[j];

			if (TestTwoObjects(obj1, obj2))
			{
				result.insert(std::make_pair(obj1, obj2));
			}
		}
	}

	return result;
}

