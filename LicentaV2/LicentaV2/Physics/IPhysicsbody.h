#pragma once

#include "CollisionTriangle.h"

namespace Physics
{
	class IPhysicsbody
	{
	public:

		virtual void FixedUpdate() = 0;
		virtual void Update() = 0;

		std::vector<Physics::CollisionTriangle *> *GetTrianglesPtr() { return &m_triangles; }

	protected:
		std::vector<Physics::CollisionTriangle *> m_triangles;
		std::vector<Rendering::VertexFormat> *m_verts;
		std::vector<unsigned int> *m_indices;
	};
}
