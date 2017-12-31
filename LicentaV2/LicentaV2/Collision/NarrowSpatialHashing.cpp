#include "NarrowSpatialHashing.h"
#include <iostream>
#include "..\Core\Utils.hpp"



Collision::NarrowSpatialHashing::NarrowSpatialHashing(float cellSize, size_t hashSize)
{
	m_cellSize = cellSize;
	m_hashSize = hashSize;

	m_pointsTable.resize(m_hashSize);
	m_pointsTable.resize(m_hashSize);

}

Collision::ContactCollection Collision::NarrowSpatialHashing::GetCollisions(std::vector<Physics::ClothNode *> points, std::vector<Physics::ClothTriangle *> triangles, std::vector<Physics::Edge *> edges)
{
	ContactCollection result;
	m_crtTimestamp = clock();

	for (auto point : points)
	{
		DeformingPoint dp;
		dp.m_node = point;
		dp.m_bb.UpdateValuesUnsorted(point->m_pos, point->m_projection);

		InsertPoint(dp);
	}

	for (auto triangle : triangles)
	{
		DeformingTriangle dt;
		dt.m_triangle = triangle;

		std::vector<glm::vec3> coords;

		dt.m_bb.UpdateValues(std::vector<glm::vec3>({ triangle->m_nodes[0]->m_pos, triangle->m_nodes[1]->m_pos, triangle->m_nodes[2]->m_pos,
			triangle->m_nodes[0]->m_projection, triangle->m_nodes[1]->m_projection, triangle->m_nodes[2]->m_projection }));


		std::vector<Physics::ClothNode *> collisions = TestTriangle(dt);

		if (collisions.size() != 0)
		{
			result.push_back(std::make_pair(dt.m_triangle, std::unordered_set<Physics::ClothNode*>(collisions.begin(), collisions.end())));
		}

		//result.insert(result.end(), collisions.begin(), collisions.end());
	}

	return result;
}


float Collision::NarrowSpatialHashing::computeTripleScalar(glm::vec3 x21, glm::vec3 x31, glm::vec3 x41, glm::vec3 v21, glm::vec3 v31, glm::vec3 v41, float t)
{
	return glm::dot(glm::cross(x21 + t * v21, x31 + t * v31), x41 + t * v41);
}

float Collision::NarrowSpatialHashing::computeTripleScalarDeriv(glm::vec3 x21, glm::vec3 x31, glm::vec3 x41, glm::vec3 v21, glm::vec3 v31, glm::vec3 v41, float t)
{
	glm::vec3 a = x21 + t * v21;
	glm::vec3 b = x31 + t * v31;
	glm::vec3 c = x41 + t * v41;

	return glm::dot(glm::cross(v21, b) + glm::cross(a, v31), c) + glm::dot(glm::cross(a, b), v41);
}

std::vector<float> Collision::NarrowSpatialHashing::NewtonRaphson(glm::vec3 x21, glm::vec3 x31, glm::vec3 x41, glm::vec3 v21, glm::vec3 v31, glm::vec3 v41, float initialGuess, int maxIterations, float tolerance)
{
	std::vector<float> result;
	float xprev = initialGuess;

	for (int i = 0; i < maxIterations; ++i)
	{
		float fCrt = computeTripleScalar(x21, x31, x41, v21, v31, v41, xprev);
		float fCrtPrime = computeTripleScalarDeriv(x21, x31, x41, v21, v31, v41, xprev);
		if (fCrtPrime == 0)
			fCrtPrime += 2 * tolerance;

		float xcrt = xprev - fCrt / fCrtPrime;

		if (std::abs(xcrt - xprev) < tolerance && xcrt >= 0 && xcrt < Core::TIME_STEP)
		{
			result.push_back(xcrt);
			//std::cout << "Found solution " << xcrt << " after " << i << " iterations." << std::endl;
			return result;
		}

		xprev = xcrt;
	}

	return result;
}

std::vector<float> Collision::NarrowSpatialHashing::FindCoplanarity(DeformingPoint p, DeformingTriangle t)
{
	glm::vec3 x1 = p.m_node->m_pos;
	glm::vec3 x2 = t.m_triangle->m_nodes[0]->m_pos;
	glm::vec3 x3 = t.m_triangle->m_nodes[1]->m_pos;
	glm::vec3 x4 = t.m_triangle->m_nodes[2]->m_pos;

	glm::vec3 v1 = p.m_node->m_vel;
	glm::vec3 v2 = t.m_triangle->m_nodes[0]->m_vel;
	glm::vec3 v3 = t.m_triangle->m_nodes[1]->m_vel;
	glm::vec3 v4 = t.m_triangle->m_nodes[2]->m_vel;

	glm::vec3 x21 = x2 - x1;
	glm::vec3 x31 = x3 - x1;
	glm::vec3 x41 = x4 - x1;

	glm::vec3 v21 = v2 - v1;
	glm::vec3 v31 = v3 - v1;
	glm::vec3 v41 = v4 - v1;


	return NewtonRaphson(x21, x31, x41, v21, v31, v41, Core::TIME_STEP / 2, 20, 1.f / 100000);
}

std::vector<Physics::ClothNode *> Collision::NarrowSpatialHashing::TestTriangle(Collision::DeformingTriangle triangle)
{
	std::vector<Physics::ClothNode *> result;

	int minX = (int)std::floor(triangle.m_bb.m_minX / m_cellSize);
	int minY = (int)std::floor(triangle.m_bb.m_minY / m_cellSize);
	int minZ = (int)std::floor(triangle.m_bb.m_minZ / m_cellSize);

	int maxX = (int)std::floor(triangle.m_bb.m_maxX / m_cellSize);
	int maxY = (int)std::floor(triangle.m_bb.m_maxY / m_cellSize);
	int maxZ = (int)std::floor(triangle.m_bb.m_maxZ / m_cellSize);

	std::unordered_set<size_t> hashIndices = GetAllIndices(minX, minY, minZ, maxX, maxY, maxZ);

	for (int index : hashIndices)
	{
		if (m_pointsTable[index].m_timestamp != m_crtTimestamp)
		{
			continue;
		}

		for (auto dp : m_pointsTable[index].m_points)
		{
			if (triangle.m_triangle->m_nodes[0] == dp.m_node || triangle.m_triangle->m_nodes[1] == dp.m_node || triangle.m_triangle->m_nodes[2] == dp.m_node)
			{
				continue;
			}
			if (triangle.m_bb.Collides(dp.m_bb))
			{

				//TODO find time of coplanarity

				//TODO elementary test

				auto coplanT = FindCoplanarity(dp, triangle);

				if (!coplanT.empty())
				{
					float t = coplanT[0];
					//result.push_back(ContactInfo({ dp.m_node, triangle.m_triangle->m_nodes[0], triangle.m_triangle->m_nodes[1], triangle.m_triangle->m_nodes[2]}));

					if (TestIntersection(dp, triangle, t))
					{
						dp.m_node->m_projection = dp.m_node->m_pos + dp.m_node->m_vel * t;
						triangle.m_triangle->m_nodes[0]->m_projection = triangle.m_triangle->m_nodes[0]->m_pos + triangle.m_triangle->m_nodes[0]->m_vel * t;
						triangle.m_triangle->m_nodes[1]->m_projection = triangle.m_triangle->m_nodes[1]->m_pos + triangle.m_triangle->m_nodes[1]->m_vel * t;
						triangle.m_triangle->m_nodes[2]->m_projection = triangle.m_triangle->m_nodes[2]->m_pos + triangle.m_triangle->m_nodes[2]->m_vel * t;
						result.push_back(dp.m_node);
					}
				}
			}
		}

	}

	return result;
}

bool Collision::NarrowSpatialHashing::TestIntersection(DeformingPoint node, DeformingTriangle triangle, float t)
{
	glm::vec3 p = node.m_node->m_pos + node.m_node->m_vel * t;
	glm::vec3 a = triangle.m_triangle->m_nodes[0]->m_pos + triangle.m_triangle->m_nodes[0]->m_vel * t - p;
	glm::vec3 b = triangle.m_triangle->m_nodes[1]->m_pos + triangle.m_triangle->m_nodes[1]->m_vel * t - p;
	glm::vec3 c = triangle.m_triangle->m_nodes[2]->m_pos + triangle.m_triangle->m_nodes[2]->m_vel * t - p;

	float ab = glm::dot(a, b);
	float ac = glm::dot(a, c);
	float bc = glm::dot(a, b);
	float cc = glm::dot(c, c);

	if (bc * ac - cc * ab < 0)
		return false;

	float bb = glm::dot(b, b);

	if (ab * bc - ac * bb < 0)
		return false;

	return true;
}

void Collision::NarrowSpatialHashing::InsertPoint(DeformingPoint dp)
{
	int minX = (int)std::floor(dp.m_bb.m_minX / m_cellSize);
	int minY = (int)std::floor(dp.m_bb.m_minY / m_cellSize);
	int minZ = (int)std::floor(dp.m_bb.m_minZ / m_cellSize);

	int maxX = (int)std::floor(dp.m_bb.m_maxX / m_cellSize);
	int maxY = (int)std::floor(dp.m_bb.m_maxY / m_cellSize);
	int maxZ = (int)std::floor(dp.m_bb.m_maxZ / m_cellSize);



// 	int xid = (int)std::floor(dp.m_node->m_projection.x / m_cellSize);
// 	int yid = (int)std::floor(dp.m_node->m_projection.y / m_cellSize);
// 	int zid = (int)std::floor(dp.m_node->m_projection.z / m_cellSize);
// 
// 	size_t id = Hash(xid, yid, zid);
// 
// 	if (m_pointsTable[id].m_timestamp != m_crtTimestamp)
// 	{
// 		m_pointsTable[id].m_points.clear();
// 		m_pointsTable[id].m_timestamp = m_crtTimestamp;
// 	}
// 
// 	m_pointsTable[id].m_points.push_back(dp);

	std::unordered_set<size_t> hashIndices = GetAllIndices(minX, minY, minZ, maxX, maxY, maxZ);

	for (int index : hashIndices)
	{
		if (m_pointsTable[index].m_timestamp != m_crtTimestamp)
		{
			m_pointsTable[index].m_points.clear();
			m_pointsTable[index].m_timestamp = m_crtTimestamp;
		}

		m_pointsTable[index].m_points.push_back(dp);
	}
}

void Collision::NarrowSpatialHashing::InsertEdge()
{

}

std::unordered_set<size_t> Collision::NarrowSpatialHashing::GetAllIndices(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
{
	std::unordered_set<size_t> result;

	for (int i = minX; i <= maxX; ++i)
	{
		result.insert(Hash(i, minY, minZ));
		result.insert(Hash(i, maxY, minZ));
		result.insert(Hash(i, minY, maxZ));
		result.insert(Hash(i, maxY, maxZ));
	}

	for (int i = minY; i <= maxY; ++i)
	{
		result.insert(Hash(minX, i, minZ));
		result.insert(Hash(maxX, i, minZ));
		result.insert(Hash(minX, i, maxZ));
		result.insert(Hash(maxX, i, maxZ));
	}

	for (int i = minZ; i <= maxZ; ++i)
	{
		result.insert(Hash(minX, minY, i));
		result.insert(Hash(maxX, minY, i));
		result.insert(Hash(minX, maxY, i));
		result.insert(Hash(maxX, maxY, i));
	}

	return result;
}

size_t Collision::NarrowSpatialHashing::Hash(int x, int y, int z)
{
	return ((x * P1) ^ (y * P2) ^ (z * P3)) % m_hashSize;
}

