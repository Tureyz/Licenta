#pragma once

#include "../Physics/ClothNode.h"
#include "../Physics/Edge.h"
#include "../Physics/ClothTriangle.h"

#include "DataStructures/BoundingBox.h"
#include "DataStructures/EdgeEdgeContactData.h"
#include "DataStructures/PointTriangleContactData.h"

#include <ctime>
#include <limits>


namespace Collision
{
	typedef struct
	{
		Physics::ClothNode *m_node;
		Collision::DataStructures::BoundingBox m_bb;
	} DeformingPoint;

	typedef struct
	{
		Physics::Edge *m_edge;
		Collision::DataStructures::BoundingBox m_bb;
	} DeformingEdge;

	typedef struct
	{
		Physics::ClothTriangle *m_triangle;
		Collision::DataStructures::BoundingBox m_bb;
	} DeformingTriangle;

	typedef struct
	{	
		clock_t m_timestamp;

		std::vector<DeformingPoint> m_points;
	} NarrowHashCellPoint;

	typedef struct
	{
		clock_t m_timestamp;

		std::vector<DeformingEdge> m_edges;
	} NarrowHashCellEdge;


	typedef std::vector<Physics::ClothNode *> ContactInfo;
	typedef std::pair<std::vector<DataStructures::PointTriangleContactData>, std::vector<DataStructures::EdgeEdgeContactData>> ContactCollection;

	class NarrowSpatialHashing
	{
	public:
		NarrowSpatialHashing(float cellSize, size_t hashSize, float h);

		ContactCollection GetCollisions(std::vector<Physics::ClothNode *> points, std::vector<Physics::ClothTriangle *> triangles, std::vector<Physics::Edge *> edges, bool continuous);
	private:
		
		

		std::vector<DataStructures::PointTriangleContactData> TestTriangle(const DeformingTriangle &triangle, bool continuous);
		std::vector<Collision::DataStructures::EdgeEdgeContactData> TestEdge(const DeformingEdge &edge, bool continuous);
		bool TestIntersection(DeformingPoint node, DeformingTriangle triangle, float t, DataStructures::PointTriangleContactData &result);
		bool TestIntersection(DeformingEdge e1, DeformingEdge e2, float t, DataStructures::EdgeEdgeContactData &result);		
		bool TestEdgeDegenerate(const glm::vec3 &x1, const glm::vec3 &x2, const glm::vec3 &x3, const glm::vec3 &x4, DataStructures::EdgeEdgeContactData &result);
		bool isValidEdgePair(const DeformingEdge &e1, const DeformingEdge &e2);
		void InsertPoint(const DeformingPoint &dp, bool continuous);
		void InsertEdge(const DeformingEdge &de, bool continuous);

		std::unordered_set<size_t> GetAllIndices(const int minX, const int minY, const int minZ, const int maxX, const int maxY, const int maxZ) const;
		std::unordered_set<size_t> GetAllIndices(const Collision::DataStructures::BoundingBox &bb) const;
		size_t Hash(const int x, const int y, const int z) const;

		const int P1 = 73856093;
		const int P2 = 19349663;
		const int P3 = 83492791;

		float m_cellSize;
		const size_t m_lowestInt = (size_t)std::abs(std::numeric_limits<int>::min());
		float m_h;
		size_t m_hashSize;
		clock_t m_crtTimestamp;
		std::vector<NarrowHashCellPoint> m_pointsTableCCD;
		std::vector<NarrowHashCellEdge> m_edgesTableCCD;
		std::vector<NarrowHashCellPoint> m_pointsTable;
		std::vector<NarrowHashCellEdge> m_edgesTable;
	};
}

