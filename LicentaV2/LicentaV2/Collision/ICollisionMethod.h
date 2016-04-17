#pragma once
#include <vector>
#include "..\Rendering\IPhysicsObject.h"

using namespace Rendering;

namespace std
{
	template <> struct hash<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> {
		inline size_t operator()(const std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *> &v) const {
			std::hash<size_t> hasher;
			return hasher(v.first->GetID()) ^ hasher(v.second->GetID());
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
		bool GetShowDebug() const { return m_showDebug; }
		void SetShowDebug(bool val) { m_showDebug = val; }
		void SetDrawBuffers(GLuint vao, GLuint vbo, GLuint ibo) { m_vao = vao; m_vbo = vbo; m_ibo = ibo; }
	protected:
		std::vector<Rendering::IPhysicsObject *> *m_allObjects;
		bool m_showDebug;
		GLuint m_vao, m_vbo, m_ibo;
	};

	inline ICollisionMethod::~ICollisionMethod() {}
}
