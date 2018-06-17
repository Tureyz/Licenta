#include "SceneObject.h"

#include "ShapeRenderer.h"

Rendering::SceneObject::SceneObject()
{
	m_translationMatrix = glm::mat4(1.0f);
	m_rotationMatrix = glm::mat4(1.0f);
	m_scaleMatrix = glm::mat4(1.0f);
	SetScale(glm::vec3(1.f));
	SetPosition(glm::vec3(0.f));
	SetRotation(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetCollisionState(DEFAULT);

	SetRotationStep(glm::vec3(0.f));
	SetRotationAngle(0.f);
	SetScaleStep(glm::vec3(1.f));
	SetTranslationStep(glm::vec3(0.f));

	m_visualBody = NULL;
	m_physicsBody = NULL;
}

Rendering::SceneObject::SceneObject(const SceneObject &other)
{
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


	m_visualBody = other.m_visualBody;

	m_boundingBox = other.m_boundingBox;
}

Rendering::SceneObject::~SceneObject()
{
	Destroy();

	if (m_auxCollisionData)
	{
		delete m_auxCollisionData;
		m_auxCollisionData = NULL;
	}

	if (m_boundingBox)
	{
		delete m_boundingBox;
		m_boundingBox = NULL;
	}

	if (m_visualBody)
	{
		delete m_visualBody;
		m_visualBody = NULL;
	}
}

void Rendering::SceneObject::Create()
{

}

void Rendering::SceneObject::Create(const glm::mat4 &mvp)
{

}

void Rendering::SceneObject::Draw()
{

}

void Rendering::SceneObject::Draw(const glm::mat4& viewProjection)
{
	if (m_visualBody)
	{
		Rendering::ShapeRenderer::Draw(viewProjection, this->m_visualBody);
	}

	if (m_boundingBox)
	{
		if (m_boundingBox->GetVisible())
		{
			Rendering::ShapeRenderer::DrawWithLines(viewProjection, this->m_boundingBox->m_visualBody);
		}
	}
}

void Rendering::SceneObject::FixedUpdate()
{
	if (GetPhysicsBody())
	{
		GetPhysicsBody()->FixedUpdate();
	}
}

void Rendering::SceneObject::Update()
{

	m_modelMatrix = m_translationMatrix * m_rotationMatrix * m_scaleMatrix;
 	m_rotationMatrix = m_scaleMatrix = glm::mat4(1);
 	m_translationMatrix = glm::mat4(1);
	UpdateVertices(m_modelMatrix);


	if (GetScaleStep() != glm::vec3(1.f))
		ScaleAbsolute(GetScaleStep() * Core::DeltaTime::GetDt());
	if (m_rotationAngleStep != 0.f)
		RotateAbsolute(GetRotationStep(), m_rotationAngleStep * Core::DeltaTime::GetDt());
	if (GetTranslationStep() != glm::vec3(0.f))
		TranslateAbsolute(GetTranslationStep() * Core::DeltaTime::GetDt());


	if (GetPhysicsBody())
	{
		GetPhysicsBody()->Update();
	}

}

void Rendering::SceneObject::Destroy()
{

}

void Rendering::SceneObject::TranslateAbsolute(const glm::vec3 &pos)
{
	m_translationMatrix = glm::translate(glm::mat4(1), pos);
	SetPosition(GetPosition() + pos);
}

void Rendering::SceneObject::RotateAbsolute(const glm::vec3 &axis, const float angles)
{
	m_rotationMatrix = glm::rotate(glm::mat4(1), angles, axis);
	SetRotation(axis);
	SetRotationAngle(angles);
}

void Rendering::SceneObject::ScaleAbsolute(const glm::vec3 &scales)
{
	m_scaleMatrix = glm::scale(glm::mat4(1), scales);
	SetScale(scales);
}

// 	 void IPhysicsObject::TranslateRelative(const glm::vec3 &pos)
// 	{
// 		m_translationMatrix = glm::translate(m_translationMatrix, pos);
// 		SetPosition(GetPosition() + pos);
// 		m_matrixChanged = true;
// 	}
// 
// 	 void IPhysicsObject::RotateRelative(const glm::vec3 &axis, const float angles)
// 	{
// 		m_rotationMatrix = glm::rotate(m_rotationMatrix, angles, axis);
// 		SetRotation(GetRotation() + axis);
// 		SetRotationAngle(GetRotationAngle() + angles);
// 		m_matrixChanged = true;
// 	}
// 
// 	 void IPhysicsObject::ScaleRelative(const glm::vec3 &scales)
// 	{
// 		m_scaleMatrix = glm::scale(m_scaleMatrix, scales);
// 		SetScale(GetScale() + scales);
// 		m_matrixChanged = true;
// 	}

void Rendering::SceneObject::UpdateVertices(glm::mat4 modelMat)
{

	if (m_visualBody)
	{
		auto bounds = m_visualBody->UpdateVerts(modelMat);

		if (m_boundingBox)
		{
			m_boundingBox->UpdateValues(bounds.first, bounds.second);
		}
	}
}

void Rendering::SceneObject::ObjectMoved()
{

}

void Rendering::SceneObject::CreateBoundingBox()
{
	m_boundingBox = new Collision::DataStructures::BoundingBox();
	m_boundingBox->CreateVisualBody(Rendering::VisualBodyFactory::GetInstance().CreateBasicVisualBody(Rendering::VisualBodyType::OBJ_LINE_CUBE));
}