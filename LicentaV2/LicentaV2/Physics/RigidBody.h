#pragma once
#include <vector>
#include <unordered_map>

#include "../Rendering/VertexFormat.h"
#include "CollisionTriangle.h"
#include "IPhysicsbody.h"


//#include "../NarrowBVH.h"




namespace Physics
{
	class RigidBody : public IPhysicsbody
	{
	public:
		RigidBody(std::vector<Rendering::VertexFormat> *verts, std::vector<unsigned int> *indices);
		~RigidBody();
		virtual void FixedUpdate() override;
		virtual void Update() override;

		//std::unordered_map<size_t, Collision::DataStructures::CollisionTriangle *> m_trianglePointers;
		//std::unordered_set<Collision::DataStructures::Edge> m_edges;

		int m_debugCnt;
		int m_debugPauseCnt;

		//Collision::NarrowBVH *m_narrowPhaseMethod;
	private:
		void CreateTriangles();
	};

}