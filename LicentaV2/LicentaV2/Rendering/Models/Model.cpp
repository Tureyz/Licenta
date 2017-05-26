#include "Model.h"
#include <algorithm>
#include "..\..\Core\Utils.hpp"
#include "../ShapeRenderer.h"

Rendering::Models::Model::Model(Managers::ModelManager *modelManager, Managers::ISimulationManager *simulationManager)
{
	m_density = 7.f;
	m_isBroken = false;
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

	SetRotationStep(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetScaleStep(glm::vec3(0.f));
	SetTranslationStep(glm::vec3(0.f));
	

	SetMass(1.f);
}

Rendering::Models::Model::Model(const Model &other)
{
	m_density = 7.f;
	m_isBroken = false;
	m_translationMatrix = other.m_translationMatrix;
	m_rotationMatrix = other.m_rotationMatrix;
	m_scaleMatrix = other.m_scaleMatrix;
	m_MVPMatrix = other.m_MVPMatrix;
	m_modelMatrix = other.m_modelMatrix;
	m_collisionState = other.m_collisionState;
	m_ID = other.m_ID;
	m_position = other.m_position;
	m_rotation = other.m_rotation;
	m_rotationAngle = other.m_rotationAngle;
	m_scale = other.m_scale;

	m_translationStep = other.m_translationStep;
	m_scaleStep = other.m_scaleStep;
	m_rotationStep = other.m_rotationStep;
	m_rotationAngleStep = other.m_rotationAngleStep;

	m_matrixChanged = other.m_matrixChanged;

	m_objectType = other.m_objectType;


	m_modelManager = other.m_modelManager;
	m_simulationManager = other.m_simulationManager;

	m_visualBody = other.m_visualBody;

	m_boundingBox = other.m_boundingBox;
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
	Create(glm::mat4(1.0f));
}

void Rendering::Models::Model::Create(const glm::mat4 &mvp)
{
	m_visualBody = m_modelManager->CreateBasicVisualBody(m_objectType);
	m_boundingBox.CreateVisualBody(m_modelManager->CreateBasicVisualBody(Simulation::PhysicsObjectType::OBJ_LINE_CUBE));

	UpdateVertices(mvp);

	SetCollisionData(new Collision::DataStructures::CollisionData(&m_visualBody.m_verts, &m_visualBody.m_transformedVerts, &m_visualBody.m_indices));
	//m_collisionData->CreateTriangles();
}

void Rendering::Models::Model::Draw()
{
}

void Rendering::Models::Model::Draw(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix)
{
	glm::mat4 MVPMatrix = projectionMatrix * viewMatrix * m_modelMatrix;
	Rendering::ShapeRenderer::Draw(MVPMatrix, this->m_visualBody);

	if (m_boundingBox.GetVisible())
	{
		Rendering::ShapeRenderer::DrawWithLines(projectionMatrix * viewMatrix, this->m_boundingBox.m_visualBody);
	}


	//DrawBB(projectionMatrix, viewMatrix);
}

void Rendering::Models::Model::Destroy()
{
}

void Rendering::Models::Model::SetBoundingBoxVisible(bool value)
{
	m_boundingBox.SetVisible(value);
}

void Rendering::Models::Model::ObjectMoved()
{
	m_modelMatrix = m_translationMatrix * m_rotationMatrix * m_scaleMatrix;
	UpdateVertices(m_modelMatrix);
	//GetBoundingBox()->UpdateValues();
	m_simulationManager->ObjectMoved(this);
}

