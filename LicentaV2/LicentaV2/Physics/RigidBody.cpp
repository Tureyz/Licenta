#include "RigidBody.h"

#include <utility>

#include "../Core/Utils.hpp"

Physics::RigidBody::RigidBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices)
{
	//m_initialVerts = initialVerts;
	m_verts = verts;
	m_indices = indices;
	m_debugCnt = 0;

	CreateTriangles();
	//std::cout <<m_verts->size() << " vertices, " << m_triangles.size() << " triangles, " << m_edges.size() << " edges" << std::endl;

	//m_narrowPhaseMethod = new Collision::NarrowBVH(&m_triangles);	
}

Physics::RigidBody::~RigidBody()
{
	for (int i = 0; i < m_triangles.size(); ++i)
	{
		delete m_triangles[i];
	}
}

void Physics::RigidBody::FixedUpdate()
{
	for (auto tri : m_triangles)
	{
		for (auto vert : tri->m_verts)
		{
			vert->m_normal = glm::vec3(0);
		}
	}

	for (auto tri : m_triangles)
	{
		tri->Update();
	}

	for (int i = 0; i < m_verts->size(); ++i)
	{
		(*m_verts)[i].m_normal = glm::normalize((*m_verts)[i].m_normal);
	}
}

void Physics::RigidBody::Update()
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

}

void Physics::RigidBody::CreateTriangles()
{
	std::vector<unsigned int> ids = *m_indices;
	std::vector<Rendering::VertexFormat> verts = *m_verts;

	
	int triangleID = 321, edgeID = 321;

	for (int i = 0; i < ids.size(); i += 3)
	{
		Physics::CollisionTriangle *triangle = new Physics::CollisionTriangle();
// 		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i]]);
// 		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i + 1]]);
// 		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i + 2]]);

		triangle->m_verts.push_back(&(*m_verts)[ids[i]]);
		triangle->m_verts.push_back(&(*m_verts)[ids[i + 1]]);
		triangle->m_verts.push_back(&(*m_verts)[ids[i + 2]]);

		triangle->m_collisionState = Rendering::CollisionState::DEFAULT;

		//Collision::DataStructures::Edge e1(triangle->m_verts[back - 2], triangle->m_verts[back - 1], edgeID++);
		//m_edges.insert(e1);
		//triangle->m_edges.push_back(e1.GetID());

		//Collision::DataStructures::Edge e2(triangle->m_verts[back - 1], triangle->m_verts[back], edgeID++);
		//m_edges.insert(e2);
		//triangle->m_edges.push_back(e2.GetID());

		//Collision::DataStructures::Edge e3(triangle->m_verts[back], triangle->m_verts[back - 2], edgeID++);
		//triangle->m_edges.push_back(e3.GetID());
		//m_edges.insert(e3);

		triangle->SetID(triangleID++);

		triangle->ComputeCenter();

		triangle->ComputeFaceNormal();

		m_triangles.push_back(triangle);
		//m_trianglePointers[triangle->GetID()] = triangle;
	}
}
