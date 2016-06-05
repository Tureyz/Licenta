#pragma once
#include <vector>
#include "..\Rendering\IPhysicsObject.h"
#include "../Benchmark/MemoryCounter.h"
#include <chrono>
#include <unordered_map>
#include <unordered_set>


using namespace Rendering;

class Rendering::IPhysicsObject;

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

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> TestCollision() final;
		virtual void Update() final;

		virtual void DrawDebug(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) = 0;

		virtual void ObjectMoved(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectAdded(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) = 0;

		void SetDrawBuffers(GLuint vao, GLuint vbo, GLuint ibo) { m_vao = vao; m_vbo = vbo; m_ibo = ibo; }
		void SetIndices(std::vector<GLuint> *indices) { m_indices = indices; }
		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }

		std::unordered_map<std::string, float> GetLastFrameCriteria() const { return m_lastFrameCriteria; }
		void SetLastFrameCriteria(std::unordered_map<std::string, float> val) { m_lastFrameCriteria = val; }

	protected:

		virtual std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> _TestCollision() = 0;
		virtual void _Update() = 0;

		std::vector<Rendering::IPhysicsObject *> *m_allObjects;
		bool m_showDebug;
		GLuint m_vao, m_vbo, m_ibo;
		std::vector<GLuint> *m_indices;
		std::unordered_map<std::string, float> m_lastFrameCriteria;

		size_t m_lastFrameTests;
		Benchmark::MemoryCounter m_memoryCounter;
	private:

	};

	inline ICollisionMethod::~ICollisionMethod() {}


	inline std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> ICollisionMethod::TestCollision()
	{		
		auto start = std::chrono::high_resolution_clock::now();
		std::unordered_set<std::pair<IPhysicsObject *, IPhysicsObject *>> result = _TestCollision();
		auto end = std::chrono::high_resolution_clock::now();

		auto timeSpent = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

		m_lastFrameCriteria["Time Spent - Collisions"] = ((float) timeSpent) / 1000000;
		m_lastFrameCriteria["Intersection Tests"] = (float)m_lastFrameTests;
		m_lastFrameCriteria["Max Memory Used (KB)"] = ((float)m_memoryCounter.GetMaxMemory()) / 1024;
		m_lastFrameCriteria["Min Memory Used (KB)"] = ((float)m_memoryCounter.GetMinMemory()) / 1024;

		return result;
	}

	inline void ICollisionMethod::Update()
	{
		auto start = std::chrono::high_resolution_clock::now();
		_Update();
		auto end = std::chrono::high_resolution_clock::now();

		auto timeSpent = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

		m_lastFrameCriteria["Time Spent - Structure Update"] = ((float) timeSpent) / 1000000;
	}

}
