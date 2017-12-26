#pragma once
#include <vector>
#include <iostream>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/freeglut/freeglut.h"

#include "../Physics/IPhysicsbody.h"
#include "../Collision/DataStructures/BoundingBox.h"
#include "../Core/DeltaTime.h"
#include "VisualBodyFactory.h"


#define VOLUME_CONSTANT 4.18879020479f


namespace Rendering
{

	class SceneObject
	{
	public:

		SceneObject();
		SceneObject(const SceneObject &other);
		~SceneObject();

		void Create();
		void Create(const glm::mat4 &mvp);
		void Draw();
		void Draw(const glm::mat4& viewProjection);
		void FixedUpdate();
		void Update();
		void Destroy();

		void TranslateAbsolute(const glm::vec3 &pos);
		void RotateAbsolute(const glm::vec3 &axis, const float angles);
		void ScaleAbsolute(const glm::vec3 &scales);

		// 		virtual void TranslateRelative(const glm::vec3 &pos);
		// 		virtual void RotateRelative(const glm::vec3 &axis, const float angles);
		// 		virtual void ScaleRelative(const glm::vec3 &scales);

		void UpdateVertices(glm::mat4 modelMat);
		void ObjectMoved();

		void CreateBoundingBox();

		bool operator==(const SceneObject &other) { return GetID() == other.GetID(); }

		// GETTERS-SETTERS
		size_t GetID() const { return m_ID; }
		void SetID(size_t val) { m_ID = val; }

		glm::vec3 GetPosition() const { return m_position; }
		void SetPosition(glm::vec3 val) { m_position = val; }

		glm::vec3 GetRotation() const { return m_rotation; }
		void SetRotation(glm::vec3 val) { m_rotation = val; }

		float GetRotationAngle() const { return m_rotationAngle; }
		void SetRotationAngle(float val) { m_rotationAngle = val; }

		glm::vec3 GetScale() const { return m_scale; }
		void SetScale(glm::vec3 val) { m_scale = val; }

		Rendering::CollisionState GetCollisionState() const { return m_collisionState; }
		void SetCollisionState(Rendering::CollisionState val) { m_collisionState = val; }

		glm::vec3 GetTranslationStep() const { return m_translationStep; }
		void SetTranslationStep(glm::vec3 val) { m_translationStep = val; }

		glm::vec3 GetScaleStep() const { return m_scaleStep; }
		void SetScaleStep(glm::vec3 val) { m_scaleStep = val; }

		glm::vec3 GetRotationStep() const { return m_rotationStep; }
		void SetRotationStep(glm::vec3 val) { m_rotationStep = val; }

		float GetRotationAngleStep() const { return m_rotationAngleStep; }
		void SetRotationAngleStep(float val) { m_rotationAngleStep = val; }

		Rendering::VisualBody *GetVisualBody() const { return m_visualBody; }
		void SetVisualBody(Rendering::VisualBody *val) { m_visualBody = val; }

		Collision::DataStructures::BoundingBox *GetBoundingBox() const { return m_boundingBox; }
		void SetBoundingBox(Collision::DataStructures::BoundingBox *val) { m_boundingBox = val; }

		Physics::IPhysicsbody *GetPhysicsBody() const { return m_physicsBody; }
		void SetPhysicsBody(Physics::IPhysicsbody *val) { m_physicsBody = val; }

		void *m_auxCollisionData;
	protected:

		glm::mat4 m_translationMatrix, m_rotationMatrix, m_scaleMatrix, m_MVPMatrix;
		glm::mat4 m_modelMatrix;

		CollisionState m_collisionState;

		size_t m_ID;

		glm::vec3 m_position;
		glm::vec3 m_rotation;
		float m_rotationAngle;
		glm::vec3 m_scale;

		glm::vec3 m_translationStep;
		glm::vec3 m_scaleStep;
		glm::vec3 m_rotationStep;
		float m_rotationAngleStep;

		Collision::DataStructures::BoundingBox *m_boundingBox;
		Physics::IPhysicsbody *m_physicsBody;
		Rendering::VisualBody *m_visualBody;

	};

}

namespace std
{
	template <> struct hash<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>>
	{
		inline size_t operator()(const std::pair<Rendering::SceneObject *, Rendering::SceneObject *> &v) const {
			std::hash<size_t> hasher;
			return hasher(v.first->GetID()) ^ hasher(v.second->GetID());
		}
	};

	template <> struct equal_to<std::pair<Rendering::SceneObject *, Rendering::SceneObject *>>
	{
		inline bool operator()(const std::pair<Rendering::SceneObject *, Rendering::SceneObject *> &l, const std::pair<Rendering::SceneObject *, Rendering::SceneObject *> &r) const
		{
			return ((l.first->GetID() == r.first->GetID()) && (r.second->GetID() == l.second->GetID())) || ((l.first->GetID() == r.second->GetID()) && (l.second->GetID() == r.first->GetID()));
		}
	};

	template <> struct equal_to<std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *>>
	{
		inline bool operator()(const std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> &l, const std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> &r) const
		{
			return ((l.first->m_position == r.first->m_position) && (r.second->m_position == l.second->m_position)) || ((l.first->m_position == r.second->m_position) && (l.second->m_position == r.first->m_position));
		}
	};

	template <> struct hash<std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *>>
	{
		inline size_t operator()(const std::pair<Rendering::VertexFormat *, Rendering::VertexFormat *> &v) const {
			std::hash<glm::vec3> hasher;
			return hasher(v.first->m_position) ^ hasher(v.second->m_position);
		}
	};


}