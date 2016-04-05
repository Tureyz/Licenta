#pragma once
#include <vector>
#include "..\Rendering\IPhysicsObject.h"

using namespace Rendering;

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
	protected:
		std::vector<Rendering::IPhysicsObject *> *m_allObjects;
		bool m_showDebug;
	};

	inline ICollisionMethod::~ICollisionMethod() {}
}
