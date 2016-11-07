#include "Model.h"
#include <algorithm>
#include "..\..\Core\Utils.hpp"
#include "../ShapeRenderer.h"

Rendering::Models::Model::Model(const glm::vec4 & color, Managers::ModelManager *modelManager, Managers::SimulationManager *simulationManager)
{
	m_translationMatrix = glm::mat4(1.0f);
	m_rotationMatrix = glm::mat4(1.0f);
	m_scaleMatrix = glm::mat4(1.0f);
	SetModelManager(modelManager);
	SetSimulationManager(simulationManager);
	SetScale(glm::vec3(1.f));
	SetPosition(glm::vec3(0.f));
	SetRotation(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetCollisionState(DEFAULT);
	SetColor(color);

	SetRotationStep(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetScaleStep(glm::vec3(0.f));
	SetTranslationStep(glm::vec3(0.f));
	

	SetMass(1.f);
}

Rendering::Models::Model::Model(const Model &other)
{
	m_translationMatrix = other.m_translationMatrix;
	m_rotationMatrix = other.m_rotationMatrix;
	m_scaleMatrix = other.m_scaleMatrix;
	m_MVPMatrix = other.m_MVPMatrix;
	m_transformedVertices = other.m_transformedVertices;
	m_initialVertices = other.m_initialVertices;
	m_indices = other.m_indices;
	m_modelMatrix = other.m_modelMatrix;
	m_color = other.m_color;
	m_collisionState = other.m_collisionState;
	m_ID = other.m_ID;
	m_position = other.m_position;
	m_rotation = other.m_rotation;
	m_rotationAngle = other.m_rotationAngle;
	m_scale = other.m_scale;
	m_verticesSize = other.m_verticesSize;

	m_translationStep = other.m_translationStep;
	m_scaleStep = other.m_scaleStep;
	m_rotationStep = other.m_rotationStep;
	m_rotationAngleStep = other.m_rotationAngleStep;

	m_minCoords = other.m_minCoords;
	m_maxCoords = other.m_maxCoords;

	m_matrixChanged = other.m_matrixChanged;

	m_objectType = other.m_objectType;


	m_vao = other.m_vao;
	m_modelManager = other.m_modelManager;
	m_simulationManager = other.m_simulationManager;
	m_vbos = other.m_vbos;

	SetBoundingBox(new Collision::DataStructures::BoundingBox(this));
}

Rendering::Models::Model::~Model()
{
	Destroy();

	if (m_auxCollisionData)
	{
		delete m_auxCollisionData;
		m_auxCollisionData = NULL;
	}
}

void Rendering::Models::Model::Create()
{

}

void Rendering::Models::Model::Draw()
{
}

void Rendering::Models::Model::DrawBB(const glm::mat4& projection_matrix, const glm::mat4& view_matrix)
{
	GetBoundingBox()->Draw(projection_matrix, view_matrix);
}

void Rendering::Models::Model::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * m_modelMatrix;
	Rendering::ShapeRenderer::Draw(MVPMatrix, m_vao, m_indices, m_collisionState);
	DrawBB(projectionMatrix, viewMatrix);
}

void Rendering::Models::Model::Destroy()
{
	delete m_boundingBox;
}

void Rendering::Models::Model::SetBoundingBoxVisible(bool value)
{
	this->GetBoundingBox()->SetVisible(value);
}

void Rendering::Models::Model::ObjectMoved()
{
	m_modelMatrix = m_translationMatrix * m_rotationMatrix * m_scaleMatrix;
	UpdateVertices(m_modelMatrix);
	GetBoundingBox()->UpdateValues();
	m_simulationManager->ObjectMoved(this);
}

