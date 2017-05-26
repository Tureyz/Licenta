#pragma once
#include <vector>

#include "../../Rendering/VertexFormat.h"

namespace Collision
{
	namespace DataStructures
	{
		class CollisionTriangle
		{
		public:

			void Update();
			static bool TriangleTest(CollisionTriangle &a, CollisionTriangle &b);
			std::vector<Rendering::VertexFormat *> m_initialVerts;
			std::vector<Rendering::VertexFormat *> m_transformedVerts;
			std::vector<std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *>> m_edges;
			Rendering::CollisionState m_collisionState;
			glm::vec3 m_center;

			glm::vec3 GetCenter();
			int GetID() const { return m_ID; }
			void SetID(int val) { m_ID = val; }

			void SetColliding();
		private:
			bool PointInTriangleTests(CollisionTriangle &other);
			static bool EdgeEdgeTests(CollisionTriangle &a, CollisionTriangle &b);


			static bool EdgeEdgeTest(std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> edge1, std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> edge2);
			static bool PointTriangleTest(Rendering::VertexFormat *point, CollisionTriangle &triangle);


			int m_ID; // uniquely identifies the triangle WITHIN ITS PARENT OBJECT
		};
	}
}