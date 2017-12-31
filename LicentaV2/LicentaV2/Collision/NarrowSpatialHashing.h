#pragma once

#include "../Physics/ClothNode.h"
#include "../Physics/Edge.h"
#include "../Physics/ClothTriangle.h"

#include "DataStructures/BoundingBox.h"
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
	typedef std::vector<std::pair<Physics::ClothTriangle *, std::unordered_set<Physics::ClothNode *>>> ContactCollection;

	class NarrowSpatialHashing
	{
	public:
		NarrowSpatialHashing(float cellSize, size_t hashSize);

		ContactCollection GetCollisions(std::vector<Physics::ClothNode *> points, std::vector<Physics::ClothTriangle *> triangles, std::vector<Physics::Edge *> edges);
	private:
		
		float computeTripleScalar(glm::vec3 x21, glm::vec3 x31, glm::vec3 x41, glm::vec3 v21, glm::vec3 v31, glm::vec3 v41, float t);
		float computeTripleScalarDeriv(glm::vec3 x21, glm::vec3 x31, glm::vec3 x41, glm::vec3 v21, glm::vec3 v31, glm::vec3 v41, float t);
		std::vector<float> NewtonRaphson(glm::vec3 x21, glm::vec3 x31, glm::vec3 x41, glm::vec3 v21, glm::vec3 v31, glm::vec3 v41, float initialGuess, int maxIterations, float tolerance);
		std::vector<float> FindCoplanarity(DeformingPoint p, DeformingTriangle t);
		std::vector<Physics::ClothNode *> TestTriangle(DeformingTriangle triangle);
		bool TestIntersection(DeformingPoint node, DeformingTriangle triangle, float t);
		void InsertPoint(DeformingPoint dp);
		void InsertEdge();

		std::unordered_set<size_t> GetAllIndices(int minX, int minY, int minZ, int maxX, int maxY, int maxZ);
		size_t Hash(int x, int y, int z);

		const int P1 = 73856093;
		const int P2 = 19349663;
		const int P3 = 83492791;

		float m_cellSize;
		const size_t m_lowestInt = (size_t)std::abs(std::numeric_limits<int>::min());;
		size_t m_hashSize;
		clock_t m_crtTimestamp;
		std::vector<NarrowHashCellPoint> m_pointsTable;
		std::vector<NarrowHashCellEdge> m_edgesTable;
	};
}

