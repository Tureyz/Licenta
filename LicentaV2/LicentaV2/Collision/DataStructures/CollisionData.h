#pragma once
#include <vector>

//#include "../../Rendering/VertexFormat.h"
#include "CollisionTriangle.h"
#include <unordered_map>


//#include "../NarrowBVH.h"




namespace Collision
{
	namespace DataStructures
	{
		class CollisionData
		{
		public:
			CollisionData(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices);
			~CollisionData();
			void FixedUpdate();
			void Update();


			//std::vector<Rendering::VertexFormat> *m_initialVerts;
			std::vector<Rendering::VertexFormat> *m_verts;
			std::vector<unsigned int> *m_indices;
			std::vector<CollisionTriangle *> m_triangles;
			std::unordered_map<size_t, CollisionTriangle *> m_trianglePointers;
			std::unordered_set<Collision::DataStructures::Edge> m_edges;

			bool m_changedSinceLastUpdate;

			int m_debugCnt;
			int m_debugPauseCnt;

			//Collision::NarrowBVH *m_narrowPhaseMethod;
		private:
			void CreateTriangles();
		};
	}
}