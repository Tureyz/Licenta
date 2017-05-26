#pragma once
#include <vector>

//#include "../../Rendering/VertexFormat.h"
#include "CollisionTriangle.h"
//#include "../NarrowBVH.h"

namespace Collision
{
	namespace DataStructures
	{
		class CollisionData
		{
		public:
			CollisionData(std::vector<Rendering::VertexFormat> *initialVerts, std::vector<Rendering::VertexFormat> *transformedVerts, std::vector<unsigned int> *indices);
			~CollisionData();
			void FixedUpdate();
			void Update();


			std::vector<Rendering::VertexFormat> *m_initialVerts;
			std::vector<Rendering::VertexFormat> *m_transformedVerts;
			std::vector<unsigned int> *m_indices;
			std::vector<CollisionTriangle *> m_triangles;

			bool m_changedSinceLastUpdate;

			int m_debugCnt;
			int m_debugPauseCnt;

			//Collision::NarrowBVH *m_narrowPhaseMethod;
		private:
			void CreateTriangles();
		};
	}
}