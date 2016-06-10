#pragma once
#include <vector>
#include <iostream>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/freeglut/freeglut.h"
#include "VertexFormat.h"


namespace Rendering
{
	enum CollisionState { DEFAULT = 0, COLLIDING = 1, ACTIVE = 2, BOUNDINGBOX = 3, COLLISIONMETHOD = 4};
	class IPhysicsObject
	{
	public:
		void *m_auxCollisionData;

		virtual ~IPhysicsObject() = 0;

		virtual void Create() = 0;
		virtual void Draw() = 0;
		virtual void Draw(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) = 0;
		virtual void DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) = 0;
		virtual void FixedUpdate() final;
		virtual void Update() final;
		virtual void Destroy() = 0;

		virtual GLuint GetVao() const final;
		virtual const std::vector<GLuint>& GetVbos() const final;

		virtual void TranslateAbsolute(const glm::vec3 &pos) final;
		virtual void RotateAbsolute(const glm::vec3 &axis, const float angles) final;
		virtual void ScaleAbsolute(const glm::vec3 &scales) final;

		virtual void TranslateRelative(const glm::vec3 &pos) final;
		virtual void RotateRelative(const glm::vec3 &axis, const float angles) final;
		virtual void ScaleRelative(const glm::vec3 &scales) final;

		virtual void UpdateVertices(glm::mat4 mvp) final;
		virtual void ObjectMoved() = 0;

		bool operator==(const IPhysicsObject &other) { return GetID() == other.GetID(); }

		// GETTERS-SETTERS
		size_t GetID() const { return m_ID; }
		void SetID(size_t val) { m_ID = val; }

		glm::vec4 GetColor() const { return m_color; }
		void SetColor(glm::vec4 val) { m_color = val; }

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

		std::vector<VertexFormat> GetVertices() { return m_transformedVertices; }
		std::vector<VertexFormat>* GetVerticesPtr() { return &m_transformedVertices; }
		void SetVertices(std::vector<VertexFormat> val) { m_transformedVertices = val; }

		glm::vec3 GetMinCoords() const { return m_minCoords; }
		void SetMinCoords(glm::vec3 val) { m_minCoords = val; }

		glm::vec3 GetMaxCoords() const { return m_maxCoords; }
		void SetMaxCoords(glm::vec3 val) { m_maxCoords = val; }

		glm::vec3 GetTranslationStep() const { return m_translationStep; }
		void SetTranslationStep(glm::vec3 val) { m_translationStep = val; }

		glm::vec3 GetScaleStep() const { return m_scaleStep; }
		void SetScaleStep(glm::vec3 val) { m_scaleStep = val; }

		glm::vec3 GetRotationStep() const { return m_rotationStep; }
		void SetRotationStep(glm::vec3 val) { m_rotationStep = val; }

		float GetRotationAngleStep() const { return m_rotationAngleStep; }
		void SetRotationAngleStep(float val) { m_rotationAngleStep = val; }

		int GetObjectType() const { return m_objectType; }
		void SetObjectType(int val) { m_objectType = val; }

		std::vector<unsigned int> GetIndices() const { return m_indices; }
		void SetIndices(std::vector<unsigned int> val) { m_indices = val; }



	protected:
		glm::mat4 m_translationMatrix, m_rotationMatrix, m_scaleMatrix, m_MVPMatrix;
		std::vector<VertexFormat> m_transformedVertices;
		std::vector<VertexFormat> m_initialVertices;
		std::vector<unsigned int> m_indices;
		glm::mat4 m_modelMatrix;
		glm::vec4 m_color;
		CollisionState m_collisionState;
		size_t m_ID;
		glm::vec3 m_position;
		glm::vec3 m_rotation;
		float m_rotationAngle;
		glm::vec3 m_scale;
		size_t m_verticesSize;

		glm::vec3 m_translationStep;
		glm::vec3 m_scaleStep;
		glm::vec3 m_rotationStep;
		float m_rotationAngleStep;

		glm::vec3 m_minCoords;
		glm::vec3 m_maxCoords;

		bool m_matrixChanged;

		int m_objectType;

		GLuint m_vao;

		std::vector<GLuint> m_vbos;

	};

	inline IPhysicsObject::~IPhysicsObject() {}

	inline void IPhysicsObject::FixedUpdate()
	{

	}

	inline void IPhysicsObject::Update()
	{
		if (m_matrixChanged)
		{
			ObjectMoved();
			m_matrixChanged = false;
		}


		if (GetScaleStep() != glm::vec3(1.f))
			ScaleRelative(GetScaleStep() /** m_modelManager->GetDt()*/);
		if (m_rotationAngleStep != 0.f)
			RotateRelative(GetRotationStep(), m_rotationAngleStep /** m_modelManager->GetDt()*/);
		if (GetTranslationStep() != glm::vec3(0.f))
			TranslateRelative(GetTranslationStep() /** m_modelManager->GetDt()*/);
	}

	inline GLuint IPhysicsObject::GetVao() const
	{
		return m_vao;
	}

	inline const std::vector<GLuint>& IPhysicsObject::GetVbos() const
	{
		return m_vbos;
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

	inline void IPhysicsObject::TranslateRelative(const glm::vec3 &pos)
	{
		m_translationMatrix = glm::translate(m_translationMatrix, pos);
		SetPosition(GetPosition() + pos);
		m_matrixChanged = true;
	}

	inline void IPhysicsObject::RotateRelative(const glm::vec3 &axis, const float angles)
	{
		m_rotationMatrix = glm::rotate(m_rotationMatrix, angles, axis);
		SetRotation(GetRotation() + axis);
		SetRotationAngle(GetRotationAngle() + angles);
		m_matrixChanged = true;
	}

	inline void IPhysicsObject::ScaleRelative(const glm::vec3 &scales)
	{
		m_scaleMatrix = glm::scale(m_scaleMatrix, scales);
		SetScale(GetScale() + scales);
		m_matrixChanged = true;
	}

	inline void IPhysicsObject::UpdateVertices(glm::mat4 mvp)
	{
		glm::vec4 asd = mvp * glm::vec4(m_initialVertices[0].m_position, 1);
		m_transformedVertices[0].m_position = glm::vec3(asd.x, asd.y, asd.z);
		m_minCoords = m_maxCoords = m_transformedVertices[0].m_position;

		for (int i = 1; i < m_initialVertices.size(); ++i)
		{
			asd = mvp * glm::vec4(m_initialVertices[i].m_position, 1);
			//std::cout << "BEFORE: " << m_vertices[i].m_position.x << " " << m_vertices[i].m_position.y << " " << m_vertices[i].m_position.z << std::endl;

			m_transformedVertices[i].m_position = glm::vec3(asd.x, asd.y, asd.z);
			if (asd.x < m_minCoords.x)
				m_minCoords.x = asd.x;
			if (asd.y < m_minCoords.y)
				m_minCoords.y = asd.y;
			if (asd.z < m_minCoords.z)
				m_minCoords.z = asd.z;

			if (asd.x > m_maxCoords.x)
				m_maxCoords.x = asd.x;
			if (asd.y > m_maxCoords.y)
				m_maxCoords.y = asd.y;
			if (asd.z > m_maxCoords.z)
				m_maxCoords.z = asd.z;
			//std::cout << "AFTER: " << m_vertices[i].m_position.x << " " << m_vertices[i].m_position.y << " " << m_vertices[i].m_position.z << std::endl;
		}
	}

}