#pragma once

#include <unordered_set>

#include "DataStructures\CollisionTriangle.h"

namespace std
{
	template <> struct hash<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>>
	{
		inline size_t operator()(const std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *> &v) const {
			std::hash<size_t> hasher;
			return hasher(v.first->GetID()) ^ hasher(v.second->GetID());
		}
	};

	template <> struct equal_to<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>>
	{
		inline bool operator()(const std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *> &l, const std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *> &r) const
		{
			return ((l.first->GetID() == r.first->GetID()) && (r.second->GetID() == l.second->GetID())) || ((l.first->GetID() == r.second->GetID()) && (l.second->GetID() == r.first->GetID()));
		}
	};
}

namespace Collision
{
	class INarrowPhaseMethod 
	{
	public:
		virtual ~INarrowPhaseMethod() = 0;

		//virtual std::unordered_set<std::pair<Collision::DataStructures::CollisionTriangle *, Collision::DataStructures::CollisionTriangle *>> TestCollision(INarrowPhaseMethod *other) = 0;
		virtual void Update() = 0;
		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) = 0;

		virtual void ObjectMoved(Collision::DataStructures::CollisionTriangle *object) = 0;
		virtual void ObjectAdded(Collision::DataStructures::CollisionTriangle *object) = 0;
		virtual void ObjectRemoved(Collision::DataStructures::CollisionTriangle *object) = 0;

		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }

	protected:
		bool m_showDebug;
		int m_id;
		std::vector<DataStructures::CollisionTriangle *> *m_allObjects;

	};

	inline INarrowPhaseMethod::~INarrowPhaseMethod() {}
}