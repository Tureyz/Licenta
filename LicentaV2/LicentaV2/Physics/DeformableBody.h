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

		std::vector<ClothNode> m_nodes;
		std::vector<Constraint> m_constraints;
		float m_density;
	private:

		std::unordered_map<size_t, std::vector<size_t>> m_vertTriangleMap;
		std::unordered_map<Physics::Edge, std::vector<size_t>> m_edgeTriangleMap;
		void CreateTriangles();
		void CreateStretchingConstraints();
		void CreateBendingConstraints();
	};

}
