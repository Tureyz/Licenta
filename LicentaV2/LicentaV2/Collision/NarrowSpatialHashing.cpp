#include "NarrowSpatialHashing.h"
#include <iostream>
#include "..\Core\Utils.hpp"
#include "../Physics/CubicSolver.h"
#include <unordered_map>



Collision::NarrowSpatialHashing::NarrowSpatialHashing(float cellSize, size_t hashSize, float h)
{
	m_cellSize = cellSize;
	m_hashSize = hashSize;
	m_h = h;

	m_pointsTableCCD.resize(m_hashSize);
	m_edgesTableCCD.resize(m_hashSize);
	m_pointsTable.resize(m_hashSize);
	m_edgesTable.resize(m_hashSize);


}

Collision::ContactCollection Collision::NarrowSpatialHashing::GetCollisions(std::vector<Physics::ClothNode *> points, std::vector<Physics::ClothTriangle *> triangles, std::vector<Physics::Edge *> edges, bool continuous)
{
	ContactCollection result;
	m_crtTimestamp = clock();

	for (auto point : points)
	{
		DeformingPoint dp;
		dp.m_node = point;
		dp.m_bb.m_thickness = m_h;
		if (continuous)
		{
			dp.m_bb.UpdateValuesUnsorted(point->m_pos, point->GetPositionInTime(Core::TIME_STEP));
		}
		else
		{
			dp.m_bb.UpdateValuesUnsorted(point->m_pos, point->m_pos);
		}

		InsertPoint(dp, continuous);
	}

	std::vector<DeformingEdge> des(edges.size());

	for (int i = 0; i < edges.size(); ++i)
	{
		auto edge = edges[i];
		DeformingEdge de;
		de.m_edge = edge;
		de.m_bb.m_thickness = m_h;
		if (continuous)
			de.m_bb.UpdateValuesUnsorted(edge->m_v1->m_pos, edge->m_v1->GetPositionInTime(Core::TIME_STEP), edge->m_v2->m_pos, edge->m_v2->GetPositionInTime(Core::TIME_STEP));
		else
			de.m_bb.UpdateValuesUnsorted(edge->m_v1->m_pos, edge->m_v2->m_pos);

		InsertEdge(de, continuous);
		des[i] = de;
	}

	std::vector<Collision::DataStructures::PointTriangleContactData> tstpt;
	std::vector<Collision::DataStructures::EdgeEdgeContactData> tstee;

	std::unordered_map<Collision::DataStructures::PointTriangleContactData, float> earliestPTContacts;

	for (auto triangle : triangles)
	{
		DeformingTriangle dt;
		dt.m_triangle = triangle;

		std::vector<glm::vec3> coords;

		glm::vec3 x1 = triangle->m_nodes[0]->m_pos;
		glm::vec3 x2 = triangle->m_nodes[1]->m_pos;
		glm::vec3 x3 = triangle->m_nodes[2]->m_pos;
		dt.m_bb.m_thickness = m_h;
		if (continuous)
		{
			glm::vec3 x1p = triangle->m_nodes[0]->GetPositionInTime(Core::TIME_STEP);
			glm::vec3 x2p = triangle->m_nodes[1]->GetPositionInTime(Core::TIME_STEP);
			glm::vec3 x3p = triangle->m_nodes[2]->GetPositionInTime(Core::TIME_STEP);

			dt.m_bb.UpdateValues(std::vector<glm::vec3>({ x1, x2, x3, x1p, x2p, x3p }));
		}
		else
		{
			dt.m_bb.UpdateValues(std::vector<glm::vec3>({ x1, x2, x3 }));
		}



		std::vector<Collision::DataStructures::PointTriangleContactData> ptCols = TestTriangle(dt, continuous);

		tstpt.insert(tstpt.end(), ptCols.begin(), ptCols.end());
		for (Collision::DataStructures::PointTriangleContactData col : ptCols)
		{
			if (earliestPTContacts.count(col) != 0)
			{
				float *tStored = &earliestPTContacts[col];
				float tCrt = col.m_t;
				*tStored = tCrt < *tStored ? tCrt : *tStored;
			}
			else
			{
				earliestPTContacts[col] = col.m_t;
			}
		}
	}

	std::unordered_map<Collision::DataStructures::EdgeEdgeContactData, float> earliestEEContacts;

	for (auto de : des)
	{
		std::vector<Collision::DataStructures::EdgeEdgeContactData> eeCols = TestEdge(de, continuous);


		tstee.insert(tstee.end(), eeCols.begin(), eeCols.end());
		for (Collision::DataStructures::EdgeEdgeContactData col : eeCols)
		{
			if (earliestEEContacts.count(col) != 0)
			{
				float *tStored = &earliestEEContacts[col];
				float tCrt = col.m_t;
				*tStored = tCrt < *tStored ? tCrt : *tStored;
			}
			else
			{
				earliestEEContacts[col] = col.m_t;
			}
		}
	}


	std::vector<Collision::DataStructures::PointTriangleContactData> ptCollisions(earliestPTContacts.size());
	std::vector<Collision::DataStructures::EdgeEdgeContactData> eeCollisions(earliestEEContacts.size());

	size_t i = 0;
	for (auto kvPair : earliestPTContacts)
	{
		ptCollisions[i++] = kvPair.first;
	}

	i = 0;
	for (auto kvPair : earliestEEContacts)
	{
		eeCollisions[i++] = kvPair.first;
	}

	return std::make_pair(ptCollisions, eeCollisions);
}









std::vector<Collision::DataStructures::PointTriangleContactData> Collision::NarrowSpatialHashing::TestTriangle(const DeformingTriangle &triangle, bool continuous)
{
	std::vector<Collision::DataStructures::PointTriangleContactData> result;

	std::unordered_set<size_t> hashIndices = GetAllIndices(triangle.m_bb);

	std::vector<NarrowHashCellPoint> &table = continuous ? m_pointsTableCCD : m_pointsTable;

	for (int index : hashIndices)
	{
		if (table[index].m_timestamp != m_crtTimestamp)
		{
			continue;
		}

		for (auto dp : table[index].m_points)
		{
			if (triangle.m_triangle->m_nodes[0] == dp.m_node || triangle.m_triangle->m_nodes[1] == dp.m_node || triangle.m_triangle->m_nodes[2] == dp.m_node)
			{
				continue;
			}
			if (continuous)
			{
				if (triangle.m_bb.Collides(dp.m_bb))
				{
					glm::vec3 x1 = dp.m_node->m_pos;
					glm::vec3 x2 = triangle.m_triangle->m_nodes[0]->m_pos;
					glm::vec3 x3 = triangle.m_triangle->m_nodes[1]->m_pos;
					glm::vec3 x4 = triangle.m_triangle->m_nodes[2]->m_pos;

					glm::vec3 v1 = dp.m_node->m_averageVel;
					glm::vec3 v2 = triangle.m_triangle->m_nodes[0]->m_averageVel;
					glm::vec3 v3 = triangle.m_triangle->m_nodes[1]->m_averageVel;
					glm::vec3 v4 = triangle.m_triangle->m_nodes[2]->m_averageVel;

					auto coplanT = Physics::CubicSolver::FindCoplanarityTimes(x1, x2, x3, x4, v1, v2, v3, v4);

					for (auto t : coplanT)
					{

						DataStructures::PointTriangleContactData pt;
						pt.m_continuous = continuous;

						if (TestIntersection(dp, triangle, t, pt))
						{
							result.push_back(pt);
							break;
						}
					}
				}
			}
			else
			{
				DataStructures::PointTriangleContactData pt;
				pt.m_continuous = continuous;

				if (TestIntersection(dp, triangle, 0, pt))
				{
					result.push_back(pt);
				}
			}
		}

	}

	return result;
}

std::vector<Collision::DataStructures::EdgeEdgeContactData> Collision::NarrowSpatialHashing::TestEdge(const DeformingEdge &edge, bool continuous)
{
	std::vector<Collision::DataStructures::EdgeEdgeContactData> result;

	std::unordered_set<size_t> hashIndices = GetAllIndices(edge.m_bb);

	std::vector<NarrowHashCellEdge> &table = continuous ? m_edgesTableCCD : m_edgesTable;

	for (int index : hashIndices)
	{
		if (table[index].m_timestamp != m_crtTimestamp)
		{
			continue;
		}

		for (auto de : table[index].m_edges)
		{
			if (!isValidEdgePair(edge, de))
			{
				continue;
			}

			if (edge.m_bb.Collides(de.m_bb))
			{
				if (continuous)
				{
					glm::vec3 x1 = de.m_edge->m_v1->m_pos;
					glm::vec3 x2 = de.m_edge->m_v2->m_pos;
					glm::vec3 x3 = edge.m_edge->m_v1->m_pos;
					glm::vec3 x4 = edge.m_edge->m_v2->m_pos;

					glm::vec3 v1 = de.m_edge->m_v1->m_averageVel;
					glm::vec3 v2 = de.m_edge->m_v2->m_averageVel;
					glm::vec3 v3 = edge.m_edge->m_v1->m_averageVel;
					glm::vec3 v4 = edge.m_edge->m_v2->m_averageVel;


					auto coplanT = Physics::CubicSolver::FindCoplanarityTimes(x1, x2, x3, x4, v1, v2, v3, v4);

					for (auto t : coplanT)
					{
						DataStructures::EdgeEdgeContactData ee;
						ee.m_continuous = continuous;

						if (TestIntersection(de, edge, t, ee))
						{
							result.push_back(ee);
							break;
						}
					}
				}
				else
				{
					DataStructures::EdgeEdgeContactData ee;
					ee.m_continuous = continuous;

					if (TestIntersection(de, edge, 0, ee))
					{
						result.push_back(ee);
					}
				}
			}
		}
	}

	return result;
}

bool Collision::NarrowSpatialHashing::TestIntersection(DeformingPoint node, DeformingTriangle triangle, float t, DataStructures::PointTriangleContactData &result)
{
	glm::vec3 x4 = node.m_node->GetPositionInTime(t);
	glm::vec3 x1 = triangle.m_triangle->m_nodes[0]->GetPositionInTime(t);
	glm::vec3 x2 = triangle.m_triangle->m_nodes[1]->GetPositionInTime(t);
	glm::vec3 x3 = triangle.m_triangle->m_nodes[2]->GetPositionInTime(t);

	glm::vec3 x13 = x1 - x3;
	glm::vec3 x23 = x2 - x3;
	glm::vec3 x43 = x4 - x3;

	glm::vec3 crs = glm::cross(x2 - x1, x3 - x1);

	glm::vec3 n = glm::normalize(crs);

	if (std::abs(glm::dot(x43, n)) < 2 * m_h)
	{

		// 		glm::mat2 m1(glm::dot(x13, x13), dTemp, dTemp, glm::dot(x23, x23));
		// 		glm::vec2 v1(glm::dot(x13, x43), glm::dot(x23, x43));
		// 
		// 		glm::vec2 w = glm::inverse(m1) * v1;


				// 	glm::mat2 m1(glm::dot(x21, x21), dTemp, dTemp, glm::dot(x43, x43));
				// 	glm::vec2 vr(glm::dot(x21, x31), glm::dot(-x43, x31));
				// 
				// 	glm::mat2 invm1 = glm::inverse(m1);
				// 	glm::vec2 ab =  invm1 * vr;
				// 
				// 	glm::vec2 wp = vr * invm1;

		float dTemp = glm::dot(x13, x23);
		float m11 = glm::dot(x13, x13);
		float m12 = dTemp;
		float m21 = dTemp;
		float m22 = glm::dot(x23, x23);

		float vr1 = glm::dot(x13, x43);
		float vr2 = glm::dot(x23, x43);

		float w2 = (m11 * vr2 - m21 * vr1) / (m11 * m22 + m21 * m12);
		float w1 = (vr1 - m12 * w2) / m11;
		float w3 = 1.f - w1 - w2;

		if (w1 < 0 || w2 < 0 || w3 < 0)
		{
			return false;
		}

		//float w3 = 1.f - w.x - w.y;

// 		if (std::abs(a - w.x) < 0.000001 || std::abs(b - w.y) < 0.000001 || std::abs(c - w3) < 0.000001)
// 		{
// 			std::cout << "WAAA\n";
// 		}

		float triangleArea = glm::length(crs) / 2.f;
		float delta = 2 * m_h / std::sqrt(triangleArea);

		if ((w1 >= -delta && w1 <= 1 + delta) && (w2 >= -delta && w2 <= 1 + delta) && (w3 >= -delta && w3 <= 1 + delta))
		{
			result.m_barycentric = glm::vec3(w1, w2, w3);
			result.m_h = m_h;

			if (n != n)
			{
				std::cout << "EEE\n";
			}
			result.m_normal = n;
			result.m_point = node.m_node;
			result.m_triangle = triangle.m_triangle;
			result.m_t = t;

			return true;
		}
	}

	return false;
}


bool Collision::NarrowSpatialHashing::TestEdgeDegenerate(const glm::vec3 &x1, const glm::vec3 &x2, const glm::vec3 &x3, const glm::vec3 &x4, DataStructures::EdgeEdgeContactData &result)
{
	float eps = 1.f / 1000000;


	glm::vec3 mid1 = (x1 + x2) / 2.f;
	glm::vec3 mid2 = (x3 + x4) / 2.f;

	if (glm::distance(mid1, mid2) < 2 * m_h)
	{
		result.m_a = 0.5;
		result.m_b = 0.5;
		result.m_normal = glm::normalize(glm::cross(x2 - x1, x4 - x3));
		return true;
	}
	// 
	// 	if (glm::distance(x1, x3) < eps)
	// 	{
	// 		result.m_a = 0;
	// 		result.m_b = 0;
	// 		result.m_normal = glm::normalize(x3 - x1);
	// 		return true;
	// 	}
	// 
	// 	if (glm::distance(x2, x3) < eps)
	// 	{
	// 		result.m_a = 1;
	// 		result.m_b = 0;
	// 		result.m_normal = glm::normalize(x3 - x2);
	// 		return true;
	// 	}
	// 
	// 	if (glm::distance(x1, x4) < eps)
	// 	{
	// 		result.m_a = 0;
	// 		result.m_b = 1;
	// 		result.m_normal = glm::normalize(x4 - x1);
	// 		return true;
	// 	}
	// 
	// 	if (glm::distance(x2, x4) < eps)
	// 	{
	// 		result.m_a = 1;
	// 		result.m_b = 1;
	// 		result.m_normal = glm::normalize(x4 - x2);
	// 		return true;
	// 	}

	return false;
}

bool Collision::NarrowSpatialHashing::isValidEdgePair(const DeformingEdge &e1, const DeformingEdge &e2)
{
	bool sameEdge = (e1.m_edge->m_v1 == e2.m_edge->m_v1 && e1.m_edge->m_v2 == e2.m_edge->m_v2) || (e1.m_edge->m_v1 == e2.m_edge->m_v2 && e1.m_edge->m_v2 == e2.m_edge->m_v1);
	bool endpointShared = e1.m_edge->m_v1 == e2.m_edge->m_v1 || e1.m_edge->m_v1 == e2.m_edge->m_v2 || e1.m_edge->m_v2 == e2.m_edge->m_v1 || e1.m_edge->m_v2 == e2.m_edge->m_v2;

	return !(sameEdge || endpointShared);
}

bool Collision::NarrowSpatialHashing::TestIntersection(DeformingEdge e1, DeformingEdge e2, float t, DataStructures::EdgeEdgeContactData &result)
{
	glm::vec3 x1 = e1.m_edge->m_v1->GetPositionInTime(t);
	glm::vec3 x2 = e1.m_edge->m_v2->GetPositionInTime(t);
	glm::vec3 x3 = e2.m_edge->m_v1->GetPositionInTime(t);
	glm::vec3 x4 = e2.m_edge->m_v2->GetPositionInTime(t);


	glm::vec3 x21 = x2 - x1;
	glm::vec3 x31 = x3 - x1;
	glm::vec3 x43 = x4 - x3;

	float eps = 1.f / 1000000;

	if (glm::length(glm::cross(x21, x43)) < eps)
	{
		if (TestEdgeDegenerate(x1, x2, x3, x4, result))
		{
			result.m_e1 = e1.m_edge;
			result.m_e2 = e2.m_edge;
			result.m_h = m_h;
			result.m_t = t;
			return true;
		}
	}

	float dTemp = glm::dot(-x21, x43);

	// 	glm::mat2 m1(glm::dot(x21, x21), dTemp, dTemp, glm::dot(x43, x43));
	// 	glm::vec2 vr(glm::dot(x21, x31), glm::dot(-x43, x31));
	// 
	// 	glm::mat2 invm1 = glm::inverse(m1);
	// 	glm::vec2 ab =  invm1 * vr;
	// 
	// 	glm::vec2 wp = vr * invm1;

	float m11 = glm::dot(x21, x21);
	float m12 = dTemp;
	float m21 = dTemp;
	float m22 = glm::dot(x43, x43);

	float vr1 = glm::dot(x21, x31);
	float vr2 = glm::dot(-x43, x31);

	float b = (m11 * vr2 - m21 * vr1) / (m11 * m22 + m21 * m12);
	float a = (vr1 - m12 * b) / m11;


	if (a >= 0 && a <= 1 && b >= 0 && b <= 1)
	{
		glm::vec3 p1 = x1 + a * x21;
		glm::vec3 p2 = x3 + b * x43;

		float dist = glm::distance(p1, p2);
		if (dist < 2 * m_h)
		{
			result.m_e1 = e1.m_edge;
			result.m_e2 = e2.m_edge;
			result.m_a = a;
			result.m_b = b;
			result.m_h = m_h;

			glm::vec3 dir = (x3 + b * x43) - (x1 + a * x21);

			if (glm::length(dir) == 0)
			{
				dir = glm::cross(x21, x43);
			}

			result.m_normal = glm::normalize(dir);// glm::normalize(glm::cross(x21, x43));
			result.m_t = t;

			return true;
		}
	}
	else
	{
		float aClamped = Core::Utils::Clamp(a, 0, 1);
		float bClamped = Core::Utils::Clamp(b, 0, 1);

		float d1 = glm::length(x21) * std::abs(a - aClamped);
		float d2 = glm::length(x43) * std::abs(b - bClamped);

		a = aClamped;
		b = bClamped;

		glm::vec3 p1, p2;
		if (d1 > d2)
		{
			p1 = x1 + a * x21;
			glm::vec3 p2Raw = Core::Utils::ProjectPointLine(p1, x3, x4);


			float distP3 = glm::distance(p2Raw, x3);
			float distP4 = glm::distance(p2Raw, x4);

			if (std::abs(distP3 + distP4 - glm::distance(x3, x4)) < eps)
			{
				p2 = p2Raw;
			}
			else
			{
				p2 = distP3 < distP4 ? x3 : x4;
			}
		}
		else
		{
			p1 = x3 + b * x43;

			glm::vec3 p2Raw = Core::Utils::ProjectPointLine(p1, x1, x2);

			float distP1 = glm::distance(p2Raw, x1);
			float distP2 = glm::distance(p2Raw, x2);

			if (std::abs(distP1 + distP2 - glm::distance(x1, x2)) < eps)
			{
				p2 = p2Raw;
			}
			else
			{
				p2 = distP1 < distP2 ? x1 : x2;
			}
		}

		if (glm::distance(p1, p2) < 2 * m_h)
		{
			result.m_a = a;
			result.m_b = b;
			result.m_e1 = e1.m_edge;
			result.m_e2 = e2.m_edge;
			result.m_h = m_h;
			result.m_normal = glm::normalize((x3 + b * x43) - (x1 + a * x21));
			result.m_t = t;
			return true;
		}
	}


	return false;
}

void Collision::NarrowSpatialHashing::InsertPoint(const DeformingPoint &dp, bool continuous)
{

	std::unordered_set<size_t> hashIndices;

	if (continuous)
	{
		hashIndices = GetAllIndices(dp.m_bb);
	}
	else
	{
		hashIndices = GetAllIndices(dp.m_bb);
		//hashIndices.insert(Hash((int)std::floor(dp.m_node->m_pos.x / m_cellSize), (int)std::floor(dp.m_node->m_pos.y / m_cellSize), (int)std::floor(dp.m_node->m_pos.z / m_cellSize)));
	}

	std::vector<NarrowHashCellPoint> &table = continuous ? m_pointsTableCCD : m_pointsTable;

	for (int index : hashIndices)
	{
		if (table[index].m_timestamp != m_crtTimestamp)
		{
			table[index].m_points.clear();
			table[index].m_timestamp = m_crtTimestamp;
		}

		table[index].m_points.push_back(dp);
	}
}

void Collision::NarrowSpatialHashing::InsertEdge(const DeformingEdge &de, bool continuous)
{
	std::unordered_set<size_t> hashIndices = GetAllIndices(de.m_bb);

	std::vector<NarrowHashCellEdge> &table = continuous ? m_edgesTableCCD : m_edgesTable;

	for (int index : hashIndices)
	{
		if (table[index].m_timestamp != m_crtTimestamp)
		{
			table[index].m_edges.clear();
			table[index].m_timestamp = m_crtTimestamp;
		}

		table[index].m_edges.push_back(de);
	}
}


std::unordered_set<size_t> Collision::NarrowSpatialHashing::GetAllIndices(const int minX, const int minY, const int minZ, const int maxX, const int maxY, const int maxZ) const
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

std::unordered_set<size_t> Collision::NarrowSpatialHashing::GetAllIndices(const Collision::DataStructures::BoundingBox &bb) const
{
	int minX = (int)std::floor(bb.m_minX / m_cellSize);
	int minY = (int)std::floor(bb.m_minY / m_cellSize);
	int minZ = (int)std::floor(bb.m_minZ / m_cellSize);
	int maxX = (int)std::floor(bb.m_maxX / m_cellSize);
	int maxY = (int)std::floor(bb.m_maxY / m_cellSize);
	int maxZ = (int)std::floor(bb.m_maxZ / m_cellSize);

	return GetAllIndices(minX, minY, minZ, maxX, maxY, maxZ);
}

size_t Collision::NarrowSpatialHashing::Hash(const int x, const int y, const int z) const
{
	return ((x * P1) ^ (y * P2) ^ (z * P3)) % m_hashSize;
}

