#pragma once

#include <vector>
#include <unordered_map>

#include "ClothNode.h"
#include "Constraint.h"
#include "IPhysicsbody.h"
#include "ClothTriangle.h"
#include "Edge.h"

#include "../Collision/NarrowSpatialHashing.h"

namespace Physics
{
	class DeformableBody : public IPhysicsbody
	{
	public:
		~DeformableBody();
		DeformableBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices);
		virtual void FixedUpdate() override;


		virtual void Update() override;


		std::vector<ClothNode *> m_nodes;
		std::vector<ClothTriangle *> m_triangles;
		std::vector<Edge *> m_edges;
		std::vector<Constraint *> m_constraints;
		std::vector<Constraint *> m_dynamicConstraints;
		float m_density;
		float m_groundHeight;
		int m_solverIterations;
		int m_collisionIterations;
		float m_kDamping;
		float m_kBending;
		float m_kStretching;
		float m_averageEdgeLength;
		float m_clothThickness;
		float m_vertexMass;

		Collision::NarrowSpatialHashing *m_selfCD;


		uint64_t m_crtStep;
		float m_totalSeconds;
	private:

		std::unordered_map<size_t, std::vector<size_t>> m_vertTriangleMap;
		std::unordered_map<Physics::Edge, Physics::ClothTriangle *> m_edgeTriangleMap;
		void CreateTriangles();		
		void CreateClothNodes(std::vector<Rendering::VertexFormat> *verts);
		void ComputeVertexMasses();
		void AdvanceVertices();

		void SolveCollisions();
		void SolveCollisionsDiscrete();
		void SolveCollisionsContinuous();

		void ApplyExternalForces();

		void UpdateTriangles();
		void AddGroundConstraints();
		void AddCollisionConstraints();
		void SolveConstraints();

		void DampVelocities();

		void CreateBendingConstraint(std::vector<ClothNode*> &t1, std::vector<ClothNode*> &t2);

		void ApplyRepulsionForce(const Collision::DataStructures::PointTriangleContactData &pt);
		void ApplyRepulsionForce(const Collision::DataStructures::EdgeEdgeContactData &ee);

		void ApplyImpulse(const Collision::DataStructures::PointTriangleContactData &pt, const float impulse);
		void ApplyImpulse(const Collision::DataStructures::EdgeEdgeContactData &ee, const float impulse);

		void ApplyRepulsionForceProj(const Collision::DataStructures::PointTriangleContactData &pt);
		void ApplyRepulsionForceProj(const Collision::DataStructures::EdgeEdgeContactData &ee);

		void ApplyImpulseProj(const Collision::DataStructures::PointTriangleContactData &pt, const float impulse);
		void ApplyImpulseProj(const Collision::DataStructures::EdgeEdgeContactData &ee, const float impulse);

		void ApplyImpulse(Physics::ClothNode *p1, Physics::ClothNode *p2, Physics::ClothNode *p3, Physics::ClothNode *p4,
			const float w1, const float w2, const float w3, const float w4, float magnitude, glm::vec3 direction);

		void ApplyRepulsionForce(Physics::ClothNode *p1, Physics::ClothNode *p2, Physics::ClothNode *p3, Physics::ClothNode *p4,
			const float w1, const float w2, const float w3, const float w4, const glm::vec3 direction, const glm::vec3 relativeVel, const float t);
	};

}
