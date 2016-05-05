#pragma once
#include <vector>
#include "..\Rendering\IPhysicsObject.h"
#include <chrono>
#include <unordered_map>


using namespace Rendering;

namespace std
{
	template <> struct hash<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>>
	{
		inline size_t operator()(const std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> &v) const {
			std::hash<size_t> hasher;
			return hasher(v.first->GetID()) ^ hasher(v.second->GetID());
		}
	};

	template <> struct equal_to<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>>
	{
		inline bool operator()(const std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> &l, const std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> &r) const
		{
			return ((l.first->GetID() == r.first->GetID()) && (r.second->GetID() == l.second->GetID())) || ((l.first->GetID() == r.second->GetID()) && (l.second->GetID() == r.first->GetID()));
		}
	};
}

namespace Collision
{
	class ICollisionMethod
	{
	public:
		virtual ~ICollisionMethod() = 0;
		virtual std::vector<Rendering::IPhysicsObject *> TestCollision(Rendering::IPhysicsObject *queriedObject) = 0;
		virtual std::vector<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() = 0;
		virtual void Update() = 0;
		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) = 0;
		virtual void ObjectMoved(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectAdded(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) = 0;
		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }
		void SetDrawBuffers(GLuint vao, GLuint vbo, GLuint ibo) { m_vao = vao; m_vbo = vbo; m_ibo = ibo; }
		std::unordered_map<std::string, float> GetLastFrameCriteria() const { return m_lastFrameCriteria; }
		void SetLastFrameCriteria(std::unordered_map<std::string, float> val) { m_lastFrameCriteria = val; }
	protected:
		std::vector<Rendering::IPhysicsObject *> *m_allObjects;
		bool m_showDebug;
		GLuint m_vao, m_vbo, m_ibo;

		std::unordered_map<std::string, float> m_lastFrameCriteria;

		size_t m_lastFrameComparisons;
	};

	inline ICollisionMethod::~ICollisionMethod() {}
}
