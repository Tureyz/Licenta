#pragma once
#include <vector>

#include "Plane.h"
#include "Edge.h"

namespace Collision
{
	namespace DataStructures
	{
		class CollisionTriangle
		{
		public:

			void Update();
			static bool TriangleTest(CollisionTriangle &a, CollisionTriangle &b);
			//std::vector<Rendering::VertexFormat *> m_initialVerts;
			std::vector<Rendering::VertexFormat *> m_verts;
			std::vector<size_t> m_edges;
			Rendering::CollisionState m_collisionState;
			glm::vec3 m_center;

			glm::vec3 m_faceNormal;
			Collision::DataStructures::Plane m_plane;

			glm::vec3 GetCenter();
			int GetID() const { return m_ID; }
			void SetID(int val) { m_ID = val; }

			void SetColliding();
			void ComputeFaceNormal();
			void ComputeCenter();
			void ComputePlane();

			glm::vec3 Barycentric(glm::vec3 p);
		private:

			void ResetCollisionState();
			void AddFaceNormalToVerts();


			static bool EdgeEdgeTest(Collision::DataStructures::Edge *edge1, Collision::DataStructures::Edge *edge2);
			static bool PointTriangleTest(glm::vec3 point, CollisionTriangle &triangle);

			static float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3);

			bool PointsSameSide(const CollisionTriangle &other);
			bool PointsSameSideDet(const CollisionTriangle &other);

			float PlaneDeterminant(const CollisionTriangle &tri, const Rendering::VertexFormat *point);

			int m_ID; // uniquely identifies the triangle WITHIN ITS PARENT OBJECT
		};
	}
}