#include "DeformableBody.h"
#include <unordered_set>
#include <iostream>
#include "../Rendering/SceneObject.h"
#include "../Core/Utils.hpp"

#include "StretchConstraint.h"
#include "BendConstraint.h"
#include "RigidConstraint.h"
#include "PointTriangleSelfConstraint.h"

#include "../Core/DeltaTime.h"




Physics::DeformableBody::~DeformableBody()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		delete m_nodes[i];
	}

	for (int i = 0; i < m_edges.size(); ++i)
	{
		delete m_edges[i];
	}
	for (int i = 0; i < m_triangles.size(); ++i)
	{
		delete m_triangles[i];
	}

	for (int i = 0; i < m_constraints.size(); ++i)
	{
		delete m_constraints[i];
	}

	delete m_selfCD;

}

Physics::DeformableBody::DeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices)
{
	m_density = 100;
	m_verts = verts;
	m_indices = indices;
	m_solverIterations = 10;
	m_groundHeight = 2.f;
	m_kStretching = 1.f;
	m_kBending = 0.02f;
	m_kDamping = 0.4f;
	m_averageEdgeLength = 0;
	m_clothThickness = 1.f / 500;

	CreateClothNodes(verts);
	CreateTriangles();

	m_averageEdgeLength /= m_edges.size();

	m_selfCD = new Collision::NarrowSpatialHashing(m_averageEdgeLength, 1999);
	ComputeVertexMasses();

	for (int i = 0; i < m_nodes.size(); i += 1560)
	{
		m_nodes[i]->m_invMass = 0;
		m_nodes[i]->m_mass = 500000;
		m_nodes[i]->m_isFixed = true;
		break;
	}

}

void Physics::DeformableBody::FixedUpdate()
{
	UpdateTriangles();
	ApplyExternalForces();
	DampVelocities();
	AdvanceVertices();

	AddCollisionConstraints();

	SolveConstraints();

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		if (!m_nodes[i]->m_isFixed)
		{
			m_nodes[i]->m_vel = (m_nodes[i]->m_projection - m_nodes[i]->m_pos) / Core::TIME_STEP;
			m_nodes[i]->m_pos = m_nodes[i]->m_projection;
			m_nodes[i]->m_vertexLink->m_position = m_nodes[i]->m_pos;
		}
	}
}

void Physics::DeformableBody::SolveConstraints()
{
	for (int i = 0; i < m_solverIterations; ++i)
	{
		for (int j = 0; j < m_constraints.size(); ++j)
		{
			m_constraints[j]->Solve(m_solverIterations);
		}

		for (int j = 0; j < m_dynamicConstraints.size(); ++j)
		{
			m_dynamicConstraints[j]->Solve(m_solverIterations);
		}
	}
}

void Physics::DeformableBody::DampVelocities()
{

	float massSum = 0;
	glm::vec3 ximiSum(0);
	glm::vec3 vimiSum(0);

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		massSum += m_nodes[i]->m_mass;
		ximiSum += m_nodes[i]->m_mass * m_nodes[i]->m_pos;
		vimiSum += m_nodes[i]->m_mass * m_nodes[i]->m_vel;
	}

	glm::vec3 xcm = ximiSum / massSum, vcm = vimiSum / massSum;

	glm::vec3 L(0);

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		L += glm::cross(m_nodes[i]->m_pos - xcm, m_nodes[i]->m_mass * m_nodes[i]->m_vel);
	}

	glm::mat3 I(0);

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		glm::vec3 ri = m_nodes[i]->m_pos - xcm;
		glm::mat3 rim(
			glm::vec3(0, -ri.z, ri.y),
			glm::vec3(ri.z, 0, -ri.x),
			glm::vec3(-ri.y, ri.x, 0));

		I += rim * glm::transpose(rim) * m_nodes[i]->m_mass;
	}

	glm::vec3 omega = glm::inverse(I) * L;

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		glm::vec3 ri = m_nodes[i]->m_pos - xcm;
		glm::vec3 deltaV = vcm + glm::cross(omega, ri) - m_nodes[i]->m_vel;
		m_nodes[i]->m_vel += m_kDamping * deltaV;
	}


}

void Physics::DeformableBody::AddGroundConstraints()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		if (m_nodes[i]->m_projection.y <= m_groundHeight)
		{
			glm::vec3 qc, nc(0, 1, 0);
			glm::vec3 p0(1, m_groundHeight, 1);

			glm::vec3 l = m_nodes[i]->m_projection - m_nodes[i]->m_vertexLink->m_position;

			float d = glm::dot((p0 - m_nodes[i]->m_projection), nc) / glm::dot(l, nc);

			qc = d * l + m_nodes[i]->m_projection;

			RigidConstraint *ct = new RigidConstraint(m_nodes[i], qc, nc, 1, false);
			ct->Init();
			m_dynamicConstraints.push_back(ct);
		}
	}
}

void Physics::DeformableBody::AddCollisionConstraints()
{
	m_dynamicConstraints.clear();

	AddGroundConstraints();

	auto selfCols = m_selfCD->GetCollisions(m_nodes, m_triangles, m_edges);

	for (auto contact : selfCols)
	{

		for (auto point : contact.first->m_nodes)
		{
			point->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;			
		}

		for (auto point : contact.second)
		{
			point->m_vertexLink->m_color = Core::COLLIDING_OBJECT_COLOR;
			PointTriangleSelfConstraint *ct = new PointTriangleSelfConstraint(std::vector<ClothNode *>({ point, contact.first->m_nodes[0], contact.first->m_nodes[1], contact.first->m_nodes[2] }), 1, false, m_clothThickness);
			ct->Init();

			m_dynamicConstraints.push_back(ct);
		}

	}
}

void Physics::DeformableBody::UpdateTriangles()
{
	for (auto tri : m_triangles)
	{
		for (auto vert : tri->m_triLink->m_verts)
		{
			vert->m_normal = glm::vec3(0);
		}
	}

	for (auto tri : m_triangles)
	{
		tri->m_triLink->Update();
	}
}

void Physics::DeformableBody::AdvanceVertices()
{
	for (int i = 0; i < m_verts->size(); ++i)
	{
		(*m_verts)[i].m_normal = glm::normalize((*m_verts)[i].m_normal);
	}

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i]->m_projection = m_nodes[i]->m_pos + m_nodes[i]->m_vel * Core::TIME_STEP;
	}
}

void Physics::DeformableBody::ApplyExternalForces()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		if (!m_nodes[i]->m_isFixed)
			m_nodes[i]->m_vel += Core::TIME_STEP * Core::GRAVITY_ACCEL / 1.f;
	}
}

void Physics::DeformableBody::Update()
{
	// 	for (int i = 0; i < m_nodes.size(); ++i)
	// 	{
	// 		if (!m_nodes[i]->m_isFixed)
	// 		{
	// 			float dt = Core::DeltaTime::GetDt();
	// 			m_nodes[i]->m_pos += m_nodes[i]->m_vel * dt;
	// 			m_nodes[i]->m_vertexLink->m_position = m_nodes[i]->m_pos;
	// 		}
	// 	}
}

void Physics::DeformableBody::CreateTriangles()
{
	std::vector<unsigned int> ids = *m_indices;

	size_t triangleID = 0, edgeID = 321;

	for (int i = 0; i < ids.size(); i += 3)
	{
		Physics::CollisionTriangle *triangle = new Physics::CollisionTriangle();
		Physics::ClothTriangle *clothTriangle = new Physics::ClothTriangle();

		m_vertTriangleMap[m_nodes[ids[i]]->m_ID].push_back(triangleID);
		m_vertTriangleMap[m_nodes[ids[i + 1]]->m_ID].push_back(triangleID);
		m_vertTriangleMap[m_nodes[ids[i + 2]]->m_ID].push_back(triangleID);

		triangle->m_verts.push_back(m_nodes[ids[i]]->m_vertexLink);
		triangle->m_verts.push_back(m_nodes[ids[i + 1]]->m_vertexLink);
		triangle->m_verts.push_back(m_nodes[ids[i + 2]]->m_vertexLink);

		triangle->m_collisionState = Rendering::CollisionState::DEFAULT;


		clothTriangle->m_nodes.push_back(m_nodes[ids[i]]);
		clothTriangle->m_nodes.push_back(m_nodes[ids[i + 1]]);
		clothTriangle->m_nodes.push_back(m_nodes[ids[i + 2]]);
		clothTriangle->m_triLink = triangle;


		//std::vector<ClothNode *> tri({ m_nodes[ids[i]], m_nodes[ids[i + 1]], m_nodes[ids[i + 2]] });

		for (int j = 0; j < 3; ++j)
		{
			ClothNode *v1 = m_nodes[ids[i + (j % 3)]], *v2 = m_nodes[ids[i + ((j + 1) % 3)]];

			Physics::Edge *e = new Physics::Edge(v1, v2, edgeID++);

			auto found = m_edgeTriangleMap.find(*e);
			if (found != m_edgeTriangleMap.end())
			{
				//(*found).second.push_back(tri);

				CreateBendingConstraint((*found).second->m_nodes, clothTriangle->m_nodes);
				m_edgeTriangleMap.erase(*e);
			}
			else
			{
				m_averageEdgeLength += glm::distance(v1->m_pos, v2->m_pos);

				m_edges.push_back(e);

				m_edgeTriangleMap[*e] = clothTriangle;
				Physics::StretchConstraint *ct = new Physics::StretchConstraint(std::vector<ClothNode *>({ v1, v2 }), m_kStretching, true);
				ct->Init();

				m_constraints.push_back(ct);
			}
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

		m_triangles.push_back(clothTriangle);
		//m_trianglePointers[triangle->GetID()] = triangle;
	}
}

void Physics::DeformableBody::CreateClothNodes(std::vector<Rendering::VertexFormat> *verts)
{
	for (int i = 0; i < verts->size(); ++i)
	{
		glm::vec3 vel(0);// = Core::Utils::RandomRangeVec(-1, 1) / 5.f;
		m_nodes.push_back(new Physics::ClothNode((*verts)[i].m_position, vel, &(*verts)[i], i));
	}
}

void Physics::DeformableBody::CreateBendingConstraint(std::vector<ClothNode*> &t1, std::vector<ClothNode*> &t2)
{
	ClothNode *p1 = NULL, *p2 = NULL, *p3 = NULL, *p4 = NULL;

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (t1[i] == t2[j])
			{
				p1 = t1[i];
				break;
			}
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (t1[i] == t2[j] && t1[i] != p1)
			{
				p2 = t1[i];
				break;
			}
		}
	}


	for (int i = 0; i < 3; ++i)
	{
		if (t1[i] != p1 && t1[i] != p2)
		{
			p3 = t1[i];
			break;
		}
	}

	for (int i = 0; i < 3; ++i)
	{
		if (t2[i] != p1 && t2[i] != p2)
		{
			p4 = t2[i];
			break;
		}
	}

	// 		std::cout << "[";
	// 		std::cout << "("<<p1->m_pos.x << ", " << p1->m_pos.y << ", " << p1->m_pos.z << ") - ";
	// 		std::cout << "("<<p3->m_pos.x << ", " << p3->m_pos.y << ", " << p3->m_pos.z << ") - ";
	// 		std::cout << "("<<p2->m_pos.x << ", " << p2->m_pos.y << ", " << p2->m_pos.z << ") - ";
	// 		std::cout << "], [";
	// 		std::cout << "("<<p1->m_pos.x << ", " << p1->m_pos.y << ", " << p1->m_pos.z << ") - ";
	// 		std::cout << "("<<p2->m_pos.x << ", " << p2->m_pos.y << ", " << p2->m_pos.z << ") - ";
	// 		std::cout << "("<<p4->m_pos.x << ", " << p4->m_pos.y << ", " << p4->m_pos.z << ") - ";
	// 		std::cout << "]" << std::endl;


// 	if (p1 == NULL || p2 == NULL || p3 == NULL || p4 == NULL)
// 	{
// 		std::cout << "AAAAAs\n";
// 	}
	Physics::BendConstraint *ct = new Physics::BendConstraint(std::vector<ClothNode*>({ p1, p2, p3, p4 }), m_kBending, true);
	ct->Init();
	m_constraints.push_back(ct);
}

void Physics::DeformableBody::ComputeVertexMasses()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		for (int j = 0; j < m_vertTriangleMap[m_nodes[i]->m_ID].size(); ++j)
		{
			m_nodes[i]->m_mass += m_density * m_triangles[m_vertTriangleMap[m_nodes[i]->m_ID][j]]->m_triLink->m_area / 3;
		}

		m_nodes[i]->m_invMass = 1.f / m_nodes[i]->m_mass;
	}
}
