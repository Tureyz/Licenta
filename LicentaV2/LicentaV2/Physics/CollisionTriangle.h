#pragma once
#include <vector>

#include "Plane.h"
#include "Edge.h"

namespace Physics
{
	class CollisionTriangle
	{
	public:

		void Update();
		static bool TriangleTest(CollisionTriangle &a, CollisionTriangle &b);
		//std::vector<Rendering::VertexFormat *> m_initialVerts;
		std::vector<Rendering::VertexFormat *> m_verts;
		Rendering::CollisionState m_collisionState;
		glm::vec3 m_center;

		glm::vec3 m_faceNormal;
		Physics::Plane m_plane;
		float m_area;

		glm::vec3 GetCenter();
		size_t GetID() const { return m_ID; }
		void SetID(size_t val) { m_ID = val; }

		void SetColliding();
		void ComputeFaceNormal();
		void ComputeCenter();
		void ComputePlane();
		void ComputeArea();

		glm::vec3 Barycentric(glm::vec3 p);
	private:

		void ResetCollisionState();
		void AddFaceNormalToVerts();


		static bool EdgeEdgeTest(Physics::Edge *edge1, Physics::Edge *edge2);
		static bool PointTriangleTest(glm::vec3 point, CollisionTriangle &triangle);

		static float TriArea2D(float x1, float y1, float x2, float y2, float x3, float y3);

		bool PointsSameSide(const CollisionTriangle &other);
		bool PointsSameSideDet(const CollisionTriangle &other);

		float PlaneDeterminant(const CollisionTriangle &tri, const Rendering::VertexFormat *point);

		size_t m_ID; // uniquely identifies the triangle WITHIN ITS PARENT OBJECT
	};
}