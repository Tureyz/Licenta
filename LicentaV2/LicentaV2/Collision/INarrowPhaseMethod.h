#pragma once

#include <unordered_set>

#include "../Physics/CollisionTriangle.h"

namespace std
{
	template <> struct hash<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>>
	{
		inline size_t operator()(const std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *> &v) const {
			std::hash<size_t> hasher;
			return hasher(v.first->GetID()) ^ hasher(v.second->GetID());
		}
	};

	template <> struct equal_to<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>>
	{
		inline bool operator()(const std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *> &l, const std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *> &r) const
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

		//virtual std::unordered_set<std::pair<Physics::CollisionTriangle *, Physics::CollisionTriangle *>> TestCollision(INarrowPhaseMethod *other) = 0;
		virtual void Update() = 0;
		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) = 0;

		virtual void ObjectMoved(Physics::CollisionTriangle *object) = 0;
		virtual void ObjectAdded(Physics::CollisionTriangle *object) = 0;
		virtual void ObjectRemoved(Physics::CollisionTriangle *object) = 0;

		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }

	protected:
		bool m_showDebug;
		int m_id;
		std::vector<Physics::CollisionTriangle *> *m_allObjects;

	};

	inline INarrowPhaseMethod::~INarrowPhaseMethod() {}
}