#pragma once

#include <vector>
#include <unordered_map>

#include "ClothNode.h"
#include "Constraint.h"
#include "IPhysicsbody.h"

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
		std::vector<Constraint *> m_constraints;
		std::vector<Constraint *> m_dynamicConstraints;
		float m_density;
		float m_groundHeight;
		int m_solverIterations;
		float m_kDamping;
		float m_kBending;
		float m_kStretching;
	private:

		std::unordered_map<size_t, std::vector<size_t>> m_vertTriangleMap;
		std::unordered_map<Physics::Edge, std::vector<ClothNode *>> m_edgeTriangleMap;
		void CreateTriangles();
		void CreateStretchingConstraints();
		void CreateBendingConstraints();
		void CreateClothNodes(std::vector<Rendering::VertexFormat> *verts);
		void ComputeVertexMasses();
		void AdvanceVertices();
		void UpdateTriangles();
		void AddGroundConstraints();
		void AddCollisionConstraints();
		void SolveConstraints();

		void DampVelocities();

		void CreateBendingConstraint(std::vector<ClothNode*> &t1, std::vector<ClothNode*> &t2);
	};

}
