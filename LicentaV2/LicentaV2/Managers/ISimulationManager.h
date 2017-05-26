#pragma once
#include <unordered_map>
#include <unordered_set>

#include "ModelManager.h"

namespace Managers
{
	class ISimulationManager
	{
	public:
		virtual ~ISimulationManager() = 0;

		virtual void Init() = 0;
		virtual void FixedUpdate() = 0;
		virtual void Update() = 0;
		virtual void Draw(const glm::mat4& projectionMatrix, const glm::mat4& viewMatrix) = 0;
		virtual void Draw() = 0;
		
		virtual void ObjectMoved(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectAdded(Rendering::IPhysicsObject *object) = 0;
		virtual void ObjectRemoved(Rendering::IPhysicsObject *object) = 0;		

		virtual void KeyPressed(unsigned char key) = 0;
		virtual void KeyReleased(unsigned char key) = 0;
		virtual void MousePressed(int button, int state, int x, int y) = 0;
		virtual void MouseMove(int x, int y, int width, int height) = 0;
		
		virtual void BreakObject(Rendering::IPhysicsObject *obj, glm::vec3 impactForce) = 0;

		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> *GetCurrentCollisionPairsPtr() { return &m_currentCollisionPairs; }
		void SetCurrentCollisionPairs(std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> &val) { m_currentCollisionPairs = val; }



	protected:
		std::unordered_set<std::pair<Rendering::IPhysicsObject *, Rendering::IPhysicsObject *>> m_currentCollisionPairs;
		ModelManager *m_modelManager;
		std::vector<Rendering::IPhysicsObject*> *m_allObjects;
		size_t m_objectIDCounter;
	};

	inline ISimulationManager::~ISimulationManager() {}
}