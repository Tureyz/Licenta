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
#include "RigidImpactZone.h"




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
	m_collisionIterations = 5;
	m_groundHeight = 4.f;
	m_kStretching = 1.0f;
	m_kBending = 0.03f;
	m_kDamping = 0.2f;
	m_averageEdgeLength = 0;
	m_vertexMass = 1.f;

	CreateClothNodes(verts);
	CreateTriangles();

	m_averageEdgeLength /= m_edges.size();
	//m_clothThickness = 1.f / 300;
	m_clothThickness = m_averageEdgeLength / 20;

	m_selfCD = new Collision::NarrowSpatialHashing(m_averageEdgeLength * 1.5, 1999, m_clothThickness);
	ComputeVertexMasses();

	for (int i = 0; i < m_nodes.size(); i += 380)
	{
		m_nodes[i]->m_invMass = 0;
		m_nodes[i]->m_mass = 50000000;
		m_nodes[i]->m_isFixed = true;
		break;
	}

}

void Physics::DeformableBody::FixedUpdate()
{
	for (int i = 0; i < 1; ++i)
	{
		UpdateTriangles();
		ApplyExternalForces();
		DampVelocities();
		AdvanceVertices();

		AddCollisionConstraints();

		SolveConstraints();
	}

	for (int i = 0; i < m_nodes.size(); ++i)
	{
		m_nodes[i]->m_averageVel = (m_nodes[i]->m_projection - m_nodes[i]->m_pos) / Core::TIME_STEP;
	}

	SolveCollisions();
	RigidImpactZone::SolveRigidImpactZones(m_selfCD, m_nodes, m_triangles, m_edges);

// 	for (int i = 0; i < m_nodes.size(); ++i)
// 	{
// 		m_nodes[i]->m_averageVel = (m_nodes[i]->m_projection - m_nodes[i]->m_pos) / Core::TIME_STEP;
// 	}



	for (int i = 0; i < m_nodes.size(); ++i)
	{
		if (!m_nodes[i]->m_isFixed)
		{
			m_nodes[i]->m_pos += m_nodes[i]->m_averageVel * Core::TIME_STEP;
			m_nodes[i]->m_vel = m_nodes[i]->m_averageVel;
			m_nodes[i]->m_vertexLink->m_position = m_nodes[i]->m_pos;
		}
	}
	//DampVelocities();
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
		//AddCollisionConstraints();
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
		m_nodes[i]->m_colliding = false;
	}
}

void Physics::DeformableBody::SolveCollisions()
{
	SolveCollisionsDiscrete();
	SolveCollisionsContinuous();
}

void Physics::DeformableBody::SolveCollisionsDiscrete()
{

	auto selfCols = m_selfCD->GetCollisions(m_nodes, m_triangles, m_edges, false);

	for (auto ptContact : selfCols.first)
	{
		//ptContact.ColorNodes();

		ApplyRepulsionForce(ptContact);

 		//ptContact.UpdateProjections();
 		//ptContact.UpdateAverageVels();

	}

	for (auto eeContact : selfCols.second)
	{
		//eeContact.ColorNodes();
		ApplyRepulsionForce(eeContact);
// 		eeContact.UpdateProjections();
 		//eeContact.UpdateAverageVels();
	}

}

void Physics::DeformableBody::SolveCollisionsContinuous()
{
	

	for (int i = 0; i < m_collisionIterations; ++i)
	{
		auto selfCols = m_selfCD->GetCollisions(m_nodes, m_triangles, m_edges, true);
		
		for (auto ptContact : selfCols.first)
		{
			//ptContact.ColorNodes();
			ApplyRepulsionForce(ptContact);

// 			ptContact.UpdateProjections();
 			//ptContact.UpdateAverageVels();			

		}

		for (auto eeContact : selfCols.second)
		{
			//eeContact.ColorNodes();
			ApplyRepulsionForce(eeContact);
// 			eeContact.UpdateProjections();
 			//eeContact.UpdateAverageVels();
		}

		if (selfCols.first.empty() && selfCols.second.empty())
		{
			break;
		}
	}
}

void Physics::DeformableBody::ApplyExternalForces()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		if (!m_nodes[i]->m_isFixed)
		{
			m_nodes[i]->m_vel += Core::TIME_STEP * Core::GRAVITY_ACCEL / 1.f;
		}
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

void Physics::DeformableBody::ApplyRepulsionForce(const Collision::DataStructures::PointTriangleContactData &pt)
{
	Physics::ClothNode *p1 = pt.m_point;
	Physics::ClothNode *p2 = pt.m_triangle->m_nodes[0];
	Physics::ClothNode *p3 = pt.m_triangle->m_nodes[1];
	Physics::ClothNode *p4 = pt.m_triangle->m_nodes[2];


	float w1 = 1;
	float w2 = pt.m_barycentric.x;
	float w3 = pt.m_barycentric.y;
	float w4 = pt.m_barycentric.z;

	glm::vec3 direction = pt.m_normal;

	glm::vec3 v1 = p1->m_averageVel;
	glm::vec3 v2 = p2->m_averageVel;
	glm::vec3 v3 = p3->m_averageVel;
	glm::vec3 v4 = p4->m_averageVel;

	glm::vec3 vel1 = v2 * w2 + v3 * w3 + v4 * w4;
	glm::vec3 vel2 = v1;

	glm::vec3 relativeVel = vel2 - vel1;

	ApplyRepulsionForce(p1, p2, p3, p4, w1, -w2, -w3, -w4, direction, relativeVel, pt.m_t);
}

void Physics::DeformableBody::ApplyRepulsionForce(const Collision::DataStructures::EdgeEdgeContactData &ee)
{
	Physics::ClothNode *p1 = ee.m_e1->m_v1;
	Physics::ClothNode *p2 = ee.m_e1->m_v2;
	Physics::ClothNode *p3 = ee.m_e2->m_v1;
	Physics::ClothNode *p4 = ee.m_e2->m_v2;

	float w1 = (1 - ee.m_a);
	float w2 = ee.m_a;
	float w3 = (1 - ee.m_b);
	float w4 = ee.m_b;

	glm::vec3 direction = ee.m_normal;

	glm::vec3 v1 = p1->m_averageVel;
	glm::vec3 v2 = p2->m_averageVel;
	glm::vec3 v3 = p3->m_averageVel;
	glm::vec3 v4 = p4->m_averageVel;

	glm::vec3 vel1 = w1 * v1 + w2 * v2;
	glm::vec3 vel2 = w3 * v3 + w4 * v4;

	glm::vec3 relativeVel = vel2 - vel1;

	ApplyRepulsionForce(p1, p2, p3, p4, -w1, -w2, w3, w4, direction, relativeVel, ee.m_t);
}

void Physics::DeformableBody::ApplyRepulsionForce(Physics::ClothNode *p1, Physics::ClothNode *p2, Physics::ClothNode *p3, Physics::ClothNode *p4, const float w1, const float w2, const float w3, const float w4, const glm::vec3 direction, const glm::vec3 relativeVel, const float t)
{

	glm::vec3 x1 = p1->GetPositionInTime(t);
	glm::vec3 x2 = p2->GetPositionInTime(t);
	glm::vec3 x3 = p3->GetPositionInTime(t);
	glm::vec3 x4 = p4->GetPositionInTime(t);

	glm::vec3 normalComponent = glm::dot(relativeVel, direction) * direction;


	int cnt = 0;

	float crtSum = 0;

	
	if (p1->m_invMass != 0)
	{
		cnt++;
		crtSum += p1->m_mass;
	}
	if (p2->m_invMass != 0)
	{
		cnt++;
		crtSum += p2->m_mass;
	}
	if (p3->m_invMass != 0)
	{
		cnt++;
		crtSum += p3->m_mass;
	}
	if (p4->m_invMass != 0)
	{
		cnt++;
		crtSum += p4->m_mass;
	}


	//float massAvg = (p1->m_mass + p2->m_mass + p3->m_mass + p4->m_mass) / 4.f;
	float massAvg = crtSum / cnt;

	float velMag = glm::length(normalComponent);

	if (glm::dot(direction, normalComponent) < 0)
	{		
		ApplyImpulse(p1, p2, p3, p4, w1, w2, w3, w4, massAvg * velMag / 2.f, direction);
	}
	else
	{
		float d = m_clothThickness - glm::dot(x1 * w1 + x2 * w2 + x3 * w3 + x4 * w4, direction);

		float frac = 1.f;
		if (velMag < frac * d / Core::TIME_STEP)
		{
			float term1 = Core::TIME_STEP * m_kStretching * d;
			float term2 = massAvg * ((frac * d / Core::TIME_STEP) - velMag);

			float ir = -(term1 < term2 ? term1 : term2);

			ApplyImpulse(p1, p2, p3, p4, w1, w2, w3, w4, ir, direction);
		}
	}
}

void Physics::DeformableBody::ApplyImpulse(Physics::ClothNode *p1, Physics::ClothNode *p2, Physics::ClothNode *p3, Physics::ClothNode *p4, const float w1, const float w2, const float w3, const float w4, float magnitude, glm::vec3 direction)
{
	float iPrime = (2.f * magnitude) / (w1 * w1 + w2 * w2 + w3 * w3 + w4 * w4);

	p1->m_averageVel += w1 * iPrime * p1->m_invMass * direction;
	p2->m_averageVel += w2 * iPrime * p2->m_invMass * direction;
	p3->m_averageVel += w3 * iPrime * p3->m_invMass * direction;
	p4->m_averageVel += w4 * iPrime * p4->m_invMass * direction;
}


///////////////////////////////////////////////////////////////////////////////////////////

void Physics::DeformableBody::ApplyRepulsionForceProj(const Collision::DataStructures::EdgeEdgeContactData &ee)
{
	glm::vec3 x1 = ee.m_e1->m_v1->GetPositionInTime(ee.m_t);
	glm::vec3 x2 = ee.m_e1->m_v2->GetPositionInTime(ee.m_t);
	glm::vec3 x3 = ee.m_e2->m_v1->GetPositionInTime(ee.m_t);
	glm::vec3 x4 = ee.m_e2->m_v2->GetPositionInTime(ee.m_t);


	glm::vec3 v1 = ee.m_e1->m_v1->m_projection - x1;
	glm::vec3 v2 = ee.m_e1->m_v2->m_projection - x2;
	glm::vec3 v3 = ee.m_e2->m_v1->m_projection - x3;
	glm::vec3 v4 = ee.m_e2->m_v2->m_projection - x4;


	glm::vec3 vel1 = (1 - ee.m_a) * v1 + ee.m_a * v2;
	glm::vec3 vel2 = (1 - ee.m_b) * v3 + ee.m_b * v4;

	glm::vec3 relativeVel = vel2 - vel1;
	glm::vec3 dist = relativeVel/* * Core::TIME_STEP*/;

	glm::vec3 normalComponent = glm::dot(dist, ee.m_normal) * ee.m_normal;

	float m1 = ee.m_e1->m_v1->m_mass;
	float m2 = ee.m_e1->m_v2->m_mass;
	float m3 = ee.m_e2->m_v1->m_mass;
	float m4 = ee.m_e2->m_v2->m_mass;

	float massAvg = (m1 + m2 + m3 + m4) / 4;


	float velMag = glm::length(normalComponent);

	if (glm::dot(ee.m_normal, normalComponent) < 0)
	{
		ApplyImpulseProj(ee, massAvg * velMag / 2.f);
	}
	else
	{
		float d = m_clothThickness - glm::dot((x3 + (x4 - x3) * ee.m_b) - (x1 + (x2 - x1) * ee.m_a), ee.m_normal);

		if (true /*velMag < 1.f * d / Core::TIME_STEP*/)
		{
			float term1 = m_kStretching * d;
			float term2 = massAvg * ((0.1f * d / Core::TIME_STEP) - velMag);

			float ir = -(term1 < term2 ? term1 : term2);

			ApplyImpulseProj(ee, ir);
		}
	}
}

void Physics::DeformableBody::ApplyImpulse(const Collision::DataStructures::PointTriangleContactData &pt, const float impulse)
{
	float w1 = pt.m_barycentric.x;
	float w2 = pt.m_barycentric.y;
	float w3 = pt.m_barycentric.z;

 	float iPrime = 2.f * impulse / (1.f + w1 * w1 + w2 * w2 + w3 * w3);
 
 	glm::vec3 term = (iPrime / m_vertexMass) * pt.m_normal;

	pt.m_triangle->m_nodes[0]->m_averageVel += w1 * term;
	pt.m_triangle->m_nodes[1]->m_averageVel += w2 * term;
	pt.m_triangle->m_nodes[2]->m_averageVel += w3 * term;

	pt.m_point->m_averageVel -= term;
}

void Physics::DeformableBody::ApplyImpulseProj(const Collision::DataStructures::PointTriangleContactData &pt, const float impulse)
{
	float w1 = pt.m_barycentric.x;
	float w2 = pt.m_barycentric.y;
	float w3 = pt.m_barycentric.z;
	float w4 = 1;

	float m1 = pt.m_triangle->m_nodes[0]->m_mass;
	float m2 = pt.m_triangle->m_nodes[1]->m_mass;
	float m3 = pt.m_triangle->m_nodes[2]->m_mass;
	float m4 = pt.m_point->m_mass;

	float iPrime = impulse / (m4 / (w4 * w4) + m1 / (w1 * w1) + m2 / (w2 * w2) + m3 / (w3 * w3));

	glm::vec3 term = pt.m_normal * iPrime;

	pt.m_triangle->m_nodes[0]->m_projection += w1 * term / m1;
	pt.m_triangle->m_nodes[1]->m_projection += w2 * term / m2;
	pt.m_triangle->m_nodes[2]->m_projection += w3 * term / m3;

	pt.m_point->m_projection -= term / m4;
}

void Physics::DeformableBody::ApplyImpulseProj(const Collision::DataStructures::EdgeEdgeContactData &ee, const float impulse)
{
	float w1 = 1 - ee.m_a;
	float w2 = ee.m_a;
	float w3 = 1 - ee.m_b;
	float w4 = ee.m_b;

	float m1 = ee.m_e1->m_v1->m_mass;
	float m2 = ee.m_e1->m_v2->m_mass;
	float m3 = ee.m_e2->m_v1->m_mass;
	float m4 = ee.m_e2->m_v2->m_mass;

	float iPrime = impulse / (m4 / (w4 * w4) + m1 / (w1 * w1) + m2 / (w2 * w2) + m3 / (w3 * w3));

	glm::vec3 term = ee.m_normal * iPrime;

	ee.m_e1->m_v1->m_projection += w1 * term / m1;
	ee.m_e1->m_v2->m_projection += w2 * term / m2;
	ee.m_e2->m_v1->m_projection -= w3 * term / m3;
	ee.m_e2->m_v2->m_projection -= w4 * term / m4;
}

void Physics::DeformableBody::ApplyRepulsionForceProj(const Collision::DataStructures::PointTriangleContactData &pt)
{
	float w1 = pt.m_barycentric.x;
	float w2 = pt.m_barycentric.y;
	float w3 = pt.m_barycentric.z;

	glm::vec3 x1 = pt.m_triangle->m_nodes[0]->GetPositionInTime(pt.m_t);
	glm::vec3 x2 = pt.m_triangle->m_nodes[1]->GetPositionInTime(pt.m_t);
	glm::vec3 x3 = pt.m_triangle->m_nodes[2]->GetPositionInTime(pt.m_t);
	glm::vec3 x4 = pt.m_point->GetPositionInTime(pt.m_t);

	glm::vec3 v1 = (pt.m_triangle->m_nodes[0]->m_projection - x1);
	glm::vec3 v2 = (pt.m_triangle->m_nodes[1]->m_projection - x2);
	glm::vec3 v3 = (pt.m_triangle->m_nodes[2]->m_projection - x3);
	glm::vec3 v4 = (pt.m_point->m_projection - x4);




	glm::vec3 vel1 = v1 * w1 + v2 * w2 + v3 * w3;
	glm::vec3 vel2 = v4;

	glm::vec3 relativeVel = vel2 - vel1;

	glm::vec3 dist = relativeVel /** Core::TIME_STEP*/;


	glm::vec3 normalComponent = glm::dot(dist, pt.m_normal) * pt.m_normal;


	float m1 = pt.m_triangle->m_nodes[0]->m_mass;
	float m2 = pt.m_triangle->m_nodes[1]->m_mass;
	float m3 = pt.m_triangle->m_nodes[2]->m_mass;
	float m4 = pt.m_point->m_mass;
	float massAvg = (m1 + m2 + m3 + m4) / 4;
	

	float velMag = glm::length(normalComponent);

	//std::cout << velMag << std::endl;

	if (glm::dot(pt.m_normal, normalComponent) < 0)
	{
		ApplyImpulseProj(pt, massAvg * velMag / 2.f);
	}
	else
	{
		float d = m_clothThickness - glm::dot(x4 - w1 * x1 - w2 * x2 - w3 * x3, pt.m_normal);

		if (velMag < 1.f * d / Core::TIME_STEP)
		{
			float term1 = m_kStretching * d;
			float term2 = massAvg * ((0.1f * d / Core::TIME_STEP) - velMag);

			float ir = -(term1 < term2 ? term1 : term2);

			ApplyImpulseProj(pt, ir);
		}
	}
}

void Physics::DeformableBody::ApplyImpulse(const Collision::DataStructures::EdgeEdgeContactData &ee, const float impulse)
{
	float a = ee.m_a;
	float aa = a * a;
	float b = ee.m_b;
	float bb = b * b;

	float a1 = 1.f - a;
	float b1 = 1.f - b;

	float iPrime = 2.f * impulse / (aa + a1 * a1 + bb + b1 * b1);


	glm::vec3 term = (iPrime / m_vertexMass) * ee.m_normal;

	ee.m_e1->m_v1->m_averageVel += a1 * term;
	ee.m_e1->m_v2->m_averageVel += a * term;

	ee.m_e2->m_v1->m_averageVel -= b1 * term;
	ee.m_e2->m_v2->m_averageVel -= b * term;
}

void Physics::DeformableBody::ComputeVertexMasses()
{
	for (int i = 0; i < m_nodes.size(); ++i)
	{
// 		for (int j = 0; j < m_vertTriangleMap[m_nodes[i]->m_ID].size(); ++j)
// 		{
// 			m_nodes[i]->m_mass += m_density * m_triangles[m_vertTriangleMap[m_nodes[i]->m_ID][j]]->m_triLink->m_area / 3;
// 		}

		m_nodes[i]->m_mass = m_vertexMass;
		m_nodes[i]->m_invMass = 1.f / m_nodes[i]->m_mass;
	}
}
