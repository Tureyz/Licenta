#include "CollisionData.h"

#include <utility>

#include "../../Core/Utils.hpp"

Collision::DataStructures::CollisionData::CollisionData(std::vector<Rendering::VertexFormat> *initialVerts, std::vector<Rendering::VertexFormat> *transformedVerts, std::vector<unsigned int> *indices)
{
	m_initialVerts = initialVerts;
	m_transformedVerts = transformedVerts;
	m_indices = indices;
	m_debugCnt = 0;

	CreateTriangles();

	//m_narrowPhaseMethod = new Collision::NarrowBVH(&m_triangles);	
}

Collision::DataStructures::CollisionData::~CollisionData()
{
	for (int i = 0; i < m_triangles.size(); ++i)
	{
		delete m_triangles[i];
	}
}

void Collision::DataStructures::CollisionData::FixedUpdate()
{
	for (auto tri : m_triangles)
	{
		tri->Update();
	}
}

void Collision::DataStructures::CollisionData::Update()
{
	
	
	if (m_debugPauseCnt++ > 30)
	{
		m_debugPauseCnt = 0;
	}
	else
	{
		return;
	}

//	int id = m_debugCnt % m_triangles.size();
// 	int prevId = (m_debugCnt - 1) % m_triangles.size();
// 
// 	m_triangles[prevId]->m_collisionState = Rendering::CollisionState::DEFAULT;
// 	for (int i = 0; i < 3; ++i)
// 	{
// 		m_triangles[prevId]->m_verts[i]->m_color = Core::DEFAULT_OBJECT_COLOR;
// 	}
// 
// 	m_triangles[id]->m_collisionState = Rendering::CollisionState::COLLIDING;
// 	for (int i = 0; i < 3; ++i)
// 	{
// 		m_triangles[id]->m_initialVerts[i]->m_color = Core::COLLIDING_OBJECT_COLOR;
// 	}
// 
// 	m_debugCnt++;

	m_changedSinceLastUpdate = true;
}

void Collision::DataStructures::CollisionData::CreateTriangles()
{
	std::vector<unsigned int> ids = *m_indices;
	std::vector<Rendering::VertexFormat> verts = *m_initialVerts;

	
	int id = 321;
	for (int i = 0; i < ids.size(); i += 3)
	{
		CollisionTriangle *triangle = new CollisionTriangle();
		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i]]);
		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i + 1]]);
		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i + 2]]);

		triangle->m_transformedVerts.push_back(&(*m_transformedVerts)[(*m_indices)[i]]);
		triangle->m_transformedVerts.push_back(&(*m_transformedVerts)[(*m_indices)[i + 1]]);
		triangle->m_transformedVerts.push_back(&(*m_transformedVerts)[(*m_indices)[i + 2]]);

		triangle->m_collisionState = Rendering::CollisionState::DEFAULT;

		int back = triangle->m_initialVerts.size() - 1;

		triangle->m_edges.push_back(std::make_pair(triangle->m_transformedVerts[back - 2], triangle->m_transformedVerts[back - 1]));
		triangle->m_edges.push_back(std::make_pair(triangle->m_transformedVerts[back - 2], triangle->m_transformedVerts[back]));
		triangle->m_edges.push_back(std::make_pair(triangle->m_transformedVerts[back - 1], triangle->m_transformedVerts[back]));

		triangle->SetID(id);

		triangle->m_center = (triangle->m_transformedVerts[0]->m_position + triangle->m_transformedVerts[1]->m_position + triangle->m_transformedVerts[2]->m_position) / 3.f;

		m_triangles.push_back(triangle);

		id++;
	}
}
