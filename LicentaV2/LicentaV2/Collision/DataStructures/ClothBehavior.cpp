#include "ClothBehavior.h"
#include <unordered_set>
#include <iostream>
#include "../../Rendering/IPhysicsObject.h"

Collision::DataStructures::ClothBehavior::ClothBehavior(Collision::DataStructures::CollisionData *data)
{

	for (int i = 0; i < data->m_verts->size(); ++i)
	{		
		m_nodes.push_back(ClothNode((*(data->m_verts))[i].m_position, glm::vec3(0), 1, &(*(data->m_verts))[i]));
	}


// 	std::unordered_set<std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *>> edges;
// 
// 	for (int i = 0; i < data->m_triangles.size(); ++i)
// 	{		
// 		edges.insert(data->m_triangles[i]->m_edges[0]);
// 		edges.insert(data->m_triangles[i]->m_edges[1]);
// 		edges.insert(data->m_triangles[i]->m_edges[2]);
// 	}
// 
// 	for (auto edge : edges)
// 	{
// 		Constraint ct;
// 		ct.m_auxValues.push_back(glm::distance(edge.first->m_position, edge.second->m_position));
// 		ct.m_cardinality = 2;
// 		ct.m_funcType = Constraint::EDGE_STRETCH;
// 		//TODO
// // 		ct.m_points.push_back(edge.first);
// // 		ct.m_points.push_back(edge.second);
// // 		m_constraints.push_back();
// 	}
// 
// 	std::cout << "edges: " << data->m_triangles.size() * 3 <<", set size: " << edges.size() << std::endl;

}
