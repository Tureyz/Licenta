#include "DeformableBody.h"
#include <unordered_set>
#include <iostream>
#include "../Rendering/SceneObject.h"
#include "../Core/Utils.hpp"




Physics::DeformableBody::~DeformableBody()
{
	for (int i = 0; i < m_triangles.size(); ++i)
	{
		delete m_triangles[i];
	}
}

Physics::DeformableBody::DeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices)
{
	m_density = 100;
	m_verts = verts;
	m_indices = indices;

	for (int i = 0; i < verts->size(); ++i)
	{
		m_nodes.push_back(Physics::ClothNode((*verts)[i].m_position, glm::vec3(0), &(*verts)[i], i));
	}

	CreateTriangles();

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		for (int j = 0; j < m_vertTriangleMap[m_nodes[i].m_ID].size(); ++j)
		{
			m_nodes[i].m_mass += m_density * m_triangles[m_vertTriangleMap[m_nodes[i].m_ID][j]]->m_area / 3;
		}

		m_nodes[i].m_invMass = 1.f / m_nodes[i].m_mass;
	}

	CreateBendingConstraints();
	CreateStretchingConstraints();


	// 	std::unordered_set<std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *>> edges;
	// 
	// 	for (int i = 0; i < data->m_triangles.size(); ++i)
	// 	{		
	// 		edges.insert(data->m_triangles[i]->m_edges[0]);
	// 		edges.insert(data->m_triangles[i]->m_edges[1]);
	// 		edges.insert(data->m_triangles[i]->m_edges[2]);
	// 	}
	// 
	// 	for (auto edge : edges)
	// 	{
	// 		Constraint ct;
	// 		ct.m_auxValues.push_back(glm::distance(edge.first->m_position, edge.second->m_position));
	// 		ct.m_cardinality = 2;
	// 		ct.m_funcType = Constraint::EDGE_STRETCH;
	// 		//TODO
	// // 		ct.m_points.push_back(edge.first);
	// // 		ct.m_points.push_back(edge.second);
	// // 		m_constraints.push_back();
	// 	}
	// 
	// 	std::cout << "edges: " << data->m_triangles.size() * 3 <<", set size: " << edges.size() << std::endl;

}

void Physics::DeformableBody::FixedUpdate()
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

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i].m_vel += Core::TIME_STEP * Core::GRAVITY_ACCEL / 100.f;
	}

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i].m_pos += m_nodes[i].m_vel * Core::TIME_STEP;
	}





	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i].m_vertexLink->m_position = m_nodes[i].m_pos;
	}
}

void Physics::DeformableBody::Update()
{
}

void Physics::DeformableBody::CreateTriangles()
{
	std::vector<unsigned int> ids = *m_indices;	

	size_t triangleID = 0, edgeID = 321;

	for (int i = 0; i < ids.size(); i += 3)
	{
		Physics::CollisionTriangle *triangle = new Physics::CollisionTriangle();
		// 		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i]]);
		// 		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i + 1]]);
		// 		triangle->m_initialVerts.push_back(&(*m_initialVerts)[(*m_indices)[i + 2]]);

		
		m_vertTriangleMap[m_nodes[ids[i]].m_ID].push_back(triangleID);
		m_vertTriangleMap[m_nodes[ids[i + 1]].m_ID].push_back(triangleID);
		m_vertTriangleMap[m_nodes[ids[i + 2]].m_ID].push_back(triangleID);

		triangle->m_verts.push_back(m_nodes[ids[i]].m_vertexLink);
		triangle->m_verts.push_back(m_nodes[ids[i + 1]].m_vertexLink);
		triangle->m_verts.push_back(m_nodes[ids[i + 2]].m_vertexLink);

		triangle->m_collisionState = Rendering::CollisionState::DEFAULT;

		
		Physics::Edge e1(&m_nodes[ids[i]], &m_nodes[ids[i + 1]], edgeID++);

		std::unordered_map<Physics::Edge, std::vector<size_t>>::iterator found = m_edgeTriangleMap.find(e1);
		if (found != m_edgeTriangleMap.end())
		{
			(*found).second.push_back(triangleID);
		}
		else
		{
			m_edgeTriangleMap[e1] = std::vector<size_t>({ triangleID });
		}

		Physics::Edge e2(&m_nodes[ids[i + 1]], &m_nodes[ids[i + 2]], edgeID++);

		found = m_edgeTriangleMap.find(e2);
		if (found != m_edgeTriangleMap.end())
		{
			(*found).second.push_back(triangleID);
		}
		else
		{
			m_edgeTriangleMap[e2] = std::vector<size_t>({ triangleID });
		}

		Physics::Edge e3(&m_nodes[ids[i + 2]], &m_nodes[ids[i]], edgeID++);

		found = m_edgeTriangleMap.find(e3);
		if (found != m_edgeTriangleMap.end())
		{
			(*found).second.push_back(triangleID);
		}
		else
		{
			m_edgeTriangleMap[e3] = std::vector<size_t>({ triangleID });
		}
		


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
		triangle->ComputeArea();

		m_triangles.push_back(triangle);
		//m_trianglePointers[triangle->GetID()] = triangle;
	}
}

void Physics::DeformableBody::CreateStretchingConstraints()
{

}

void Physics::DeformableBody::CreateBendingConstraints()
{

}
