#pragma once
#include <vector>
#include <iostream>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/freeglut/freeglut.h"

#include "../Collision/DataStructures/CollisionData.h"
#include "../Collision/DataStructures/BoundingBox.h"
#include "../Core/DeltaTime.h"


#define VOLUME_CONSTANT 4.18879020479f

namespace Simulation
{
	enum PhysicsObjectType { OBJ_CUBE = 0, OBJ_SPHERE = 1, OBJ_TETRAHEDRON = 2, OBJ_CYLINDER = 3, OBJ_CONE = 4, OBJ_MESH = 5, OBJ_LINE_CUBE = 6, OBJ_RANDOM, OBJ_NUM_TOTAL = 7 };
}


namespace Rendering
{
	
	class IPhysicsObject
	{
	public:
		void *m_auxCollisionData;

		virtual ~IPhysicsObject() = 0;

		virtual void Create() = 0;
		virtual void Create(const glm::mat4 &mvp) = 0;
		virtual void Draw() = 0;
		virtual void Draw(const glm::mat4& viewProjection) = 0;
		virtual void FixedUpdate() final;
		virtual void Update() final;
		virtual void Destroy() = 0;

		virtual void TranslateAbsolute(const glm::vec3 &pos) final;
		virtual void RotateAbsolute(const glm::vec3 &axis, const float angles) final;
		virtual void ScaleAbsolute(const glm::vec3 &scales) final;

// 		virtual void TranslateRelative(const glm::vec3 &pos) final;
// 		virtual void RotateRelative(const glm::vec3 &axis, const float angles) final;
// 		virtual void ScaleRelative(const glm::vec3 &scales) final;

		virtual void UpdateVertices(glm::mat4 modelMat) final;
		virtual void ObjectMoved() = 0;

		bool operator==(const IPhysicsObject &other) { return GetID() == other.GetID(); }

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

		float GetMass() const { return m_mass; }
		float GetInverseMass() const { return m_inverseMass; }
		void SetMass(float val) { m_mass = val; }

		float GetSphereRadius() const { return m_sphereRadius; }
		void SetSphereRadius(float val) { m_sphereRadius = val; }

		virtual bool SphereTest(Rendering::IPhysicsObject *other) final;

		bool GetBroken() const { return m_isBroken; }
		void SetBroken(bool val) { m_isBroken = val; }
		float GetDensity() const { return m_density; }
		void SetDensity(float val) { m_density = val; }
		glm::mat3 GetInverseInertiaTensor() const { return m_inverseInertiaTensor; }
		void SetInverseInertiaTensor(glm::mat3 val) { m_inverseInertiaTensor = val; }
		Simulation::PhysicsObjectType GetObjectType() const { return m_objectType; }
		void SetObjectType(Simulation::PhysicsObjectType val) { m_objectType = val; }
		Rendering::VisualBody GetVisualBody() const { return m_visualBody; }
		void SetVisualBody(Rendering::VisualBody val) { m_visualBody = val; }
		Collision::DataStructures::BoundingBox GetBoundingBox() const { return m_boundingBox; }
		void SetBoundingBox(Collision::DataStructures::BoundingBox val) { m_boundingBox = val; }
		Collision::DataStructures::CollisionData * GetCollisionData() const { return m_collisionData; }
		void SetCollisionData(Collision::DataStructures::CollisionData * val) { m_collisionData = val; }
		// 		Collision::DataStructures::CollisionData * GetCollisionData() const { return m_collisionData; }
// 		void SetCollisionData(Collision::DataStructures::CollisionData * val) { m_collisionData = val; }
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

		bool m_matrixChanged;

		Simulation::PhysicsObjectType m_objectType;

		float m_mass;
		float m_inverseMass;
		float m_sphereRadius;
		float m_density;

		glm::mat3 m_inverseInertiaTensor;

		bool m_isBroken;

		Collision::DataStructures::BoundingBox m_boundingBox;

		//MASTER
		Collision::DataStructures::CollisionData *m_collisionData;

		Rendering::VisualBody m_visualBody;

	};

	inline IPhysicsObject::~IPhysicsObject() {}

	inline void IPhysicsObject::FixedUpdate()
	{
		if (GetCollisionData())
		{
			GetCollisionData()->FixedUpdate();
			if (GetCollisionData()->m_changedSinceLastUpdate)
			{
			}
		}
	}

	inline void IPhysicsObject::Update()
	{

		if (m_matrixChanged)
		{
			ObjectMoved();
			m_matrixChanged = false;

		}


		if (GetScaleStep() != glm::vec3(1.f))
			ScaleAbsolute(GetScaleStep() * Core::DeltaTime::GetDt());
		if (m_rotationAngleStep != 0.f)
			RotateAbsolute(GetRotationStep(), m_rotationAngleStep * Core::DeltaTime::GetDt());
		if (GetTranslationStep() != glm::vec3(0.f))
			TranslateAbsolute(GetTranslationStep() * Core::DeltaTime::GetDt());


		if (GetCollisionData())
		{
			GetCollisionData()->Update();
 			if (GetCollisionData()->m_changedSinceLastUpdate)
 			{
 			}
		}
		
	}

	inline void IPhysicsObject::TranslateAbsolute(const glm::vec3 &pos)
	{
		m_translationMatrix = glm::translate(glm::mat4(1), pos);
		SetPosition(pos);
		m_matrixChanged = true;
	}

	inline void IPhysicsObject::RotateAbsolute(const glm::vec3 &axis, const float angles)
	{
		m_rotationMatrix = glm::rotate(glm::mat4(1), angles, axis);
		SetRotation(axis);
		SetRotationAngle(angles);
		m_matrixChanged = true;
	}

	inline void IPhysicsObject::ScaleAbsolute(const glm::vec3 &scales)
	{
		m_scaleMatrix = glm::scale(glm::mat4(1), scales);
		SetScale(scales);
		m_matrixChanged = true;
	}

// 	inline void IPhysicsObject::TranslateRelative(const glm::vec3 &pos)
// 	{
// 		m_translationMatrix = glm::translate(m_translationMatrix, pos);
// 		SetPosition(GetPosition() + pos);
// 		m_matrixChanged = true;
// 	}
// 
// 	inline void IPhysicsObject::RotateRelative(const glm::vec3 &axis, const float angles)
// 	{
// 		m_rotationMatrix = glm::rotate(m_rotationMatrix, angles, axis);
// 		SetRotation(GetRotation() + axis);
// 		SetRotationAngle(GetRotationAngle() + angles);
// 		m_matrixChanged = true;
// 	}
// 
// 	inline void IPhysicsObject::ScaleRelative(const glm::vec3 &scales)
// 	{
// 		m_scaleMatrix = glm::scale(m_scaleMatrix, scales);
// 		SetScale(GetScale() + scales);
// 		m_matrixChanged = true;
// 	}

	inline void IPhysicsObject::UpdateVertices(glm::mat4 modelMat)
	{

		auto bounds = m_visualBody.UpdateVerts(modelMat);
		m_boundingBox.UpdateValues(bounds.first, bounds.second);			


// 		glm::vec4 asd = mvp * glm::vec4(m_initialVertices[0].m_position, 1);
// 		m_transformedVertices[0].m_position = glm::vec3(asd.x, asd.y, asd.z);
// 		m_minCoords = m_maxCoords = m_transformedVertices[0].m_position;
// 		
// 
// 		float max = 0.0f;
// 		for (int i = 1; i < m_initialVertices.size(); ++i)
// 		{
// 			asd = mvp * glm::vec4(m_initialVertices[i].m_position, 1);
// 			//std::wcout << "BEFORE: " << m_vertices[i].m_position.x << " " << m_vertices[i].m_position.y << " " << m_vertices[i].m_position.z << std::endl;
// 
// 			m_transformedVertices[i].m_position = glm::vec3(asd.x, asd.y, asd.z);
// 			if (asd.x < m_minCoords.x)
// 				m_minCoords.x = asd.x;
// 			if (asd.y < m_minCoords.y)
// 				m_minCoords.y = asd.y;
// 			if (asd.z < m_minCoords.z)
// 				m_minCoords.z = asd.z;
// 
// 			if (asd.x > m_maxCoords.x)
// 				m_maxCoords.x = asd.x;
// 			if (asd.y > m_maxCoords.y)
// 				m_maxCoords.y = asd.y;
// 			if (asd.z > m_maxCoords.z)
// 				m_maxCoords.z = asd.z;
// 
// 			auto crtDist = glm::distance(m_position, m_transformedVertices[i].m_position);
// 
// 			if (crtDist > max)
// 			{
// 				max = crtDist;
// 			}
// 			//std::wcout << "AFTER: " << m_vertices[i].m_position.x << " " << m_vertices[i].m_position.y << " " << m_vertices[i].m_position.z << std::endl;
// 		}
// 
// 		
// 
// 		m_sphereRadius = max;
// 		SetMass(m_sphereRadius * m_sphereRadius * m_sphereRadius * VOLUME_CONSTANT * m_density);
// 		m_inverseMass = 1.f / m_mass;
// 		m_inverseInertiaTensor = glm::mat3(1) * (1.f / (0.4f * m_mass * m_sphereRadius * m_sphereRadius));
	}

	inline bool IPhysicsObject::SphereTest(Rendering::IPhysicsObject *other)
	{
		return glm::distance(this->GetPosition(), other->GetPosition()) < this->GetSphereRadius() + other->GetSphereRadius();
	}

}

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