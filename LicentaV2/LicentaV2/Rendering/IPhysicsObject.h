#pragma once
#include <vector>
#include <iostream>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/freeglut/freeglut.h"
#include "VertexFormat.h"


namespace Rendering
{
	enum CollisionState { DEFAULT = 0, COLLIDING = 1, ACTIVE = 2, BOUNDINGBOX = 3 };
	class IPhysicsObject
	{
	public:
		virtual ~IPhysicsObject() = 0;

		virtual void Draw() = 0;
		virtual void Draw(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) = 0;
		virtual void DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix) = 0;
		virtual void Update() = 0;
		virtual void SetProgram(GLuint shaderName) = 0;
		virtual void Destroy() = 0;

		virtual GLuint GetVao() const = 0;
		virtual const std::vector<GLuint>& GetVbos() const = 0;

		virtual void TranslateAbsolute(const glm::vec3 &pos) = 0;
		virtual void RotateAbsolute(const glm::vec3 &axis, const float angles) = 0;
		virtual void ScaleAbsolute(const glm::vec3 &scales) = 0;

		virtual void TranslateRelative(const glm::vec3 &pos) = 0;
		virtual void RotateRelative(const glm::vec3 &axis, const float angles) = 0;
		virtual void ScaleRelative(const glm::vec3 &scales) = 0;

		unsigned long GetID() const { return m_ID; }
		void SetID(unsigned long val) { m_ID = val; }

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
		virtual void UpdateVertices(glm::mat4 mvp) = 0;

		glm::vec3 GetMinCoords() const { return m_minCoords; }
		void SetMinCoords(glm::vec3 val) { m_minCoords = val; }
		glm::vec3 GetMaxCoords() const { return m_maxCoords; }
		void SetMaxCoords(glm::vec3 val) { m_maxCoords = val; }
	protected:
		glm::mat4 m_translationMatrix, m_rotationMatrix, m_scaleMatrix, m_MVPMatrix;
		std::vector<VertexFormat> m_transformedVertices;
		std::vector<VertexFormat> m_initialVertices;
		glm::mat4 m_modelMatrix;
		glm::vec4 m_color;
		CollisionState m_collisionState;
		unsigned long m_ID;
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
	};

	inline IPhysicsObject::~IPhysicsObject() {}
}