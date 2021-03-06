#include "ModelManager.h"

#include <utility>
#include <string>
#include <cstdlib>

Managers::ModelManager::ModelManager()
{

}

Managers::ModelManager::~ModelManager()
{
	for (auto model : m_objectList)
	{
		delete model;
	}
	m_objectList.clear();

	for (auto model : m_sfxObjectList)
	{
		delete model;
	}
	m_sfxObjectList.clear();
}

void Managers::ModelManager::FixedUpdate()
{
	for (auto model : m_objectList)
	{
		model->FixedUpdate();
	}

	for (auto model : m_sfxObjectList)
	{
		model->FixedUpdate();
	}
}

void Managers::ModelManager::Draw(const glm::mat4& viewProjection)
{
	for (auto model : m_objectList)
	{
		model->Draw(viewProjection);
		//std::wcout << model->GetPosition().x << L" " << model->GetPosition().y << L" " << model->GetPosition().z << std::endl;

	}

	for (auto model : m_sfxObjectList)
	{
		model->Draw(viewProjection);
		//std::wcout << model->GetPosition().x << L" " << model->GetPosition().y << L" " << model->GetPosition().z << std::endl;

	}
}

void Managers::ModelManager::Update()
{
	for (auto model : m_objectList)
	{
		model->Update();
	}

	for (auto model : m_sfxObjectList)
	{
		model->Update();
	}
}

void Managers::ModelManager::DeleteAllModels()
{
	if (!m_objectList.empty())
	{
		for (auto model : m_objectList)
		{
			delete model;
		}

		m_objectList.clear();
	}

	if (!m_sfxObjectList.empty())
	{
		for (auto model : m_sfxObjectList)
		{
			delete model;
		}

		m_sfxObjectList.clear();
	}
}

void Managers::ModelManager::DeleteModel(std::size_t id)
{
	Rendering::SceneObject *model = NULL;
	for (auto it = m_objectList.begin(); it != m_objectList.end(); ++it)
	{
		if ((*it)->GetID() == id)
		{
			(*it)->Destroy();
			m_objectList.erase(it);
			break;
		}
	}

	for (auto it = m_sfxObjectList.begin(); it != m_sfxObjectList.end(); ++it)
	{
		if ((*it)->GetID() == id)
		{
			(*it)->Destroy();
			m_sfxObjectList.erase(it);
			break;
		}
	}
}

const Rendering::SceneObject* Managers::ModelManager::GetModel(unsigned long id) const
{
	for (auto mod : m_objectList)
	{
		if (mod->GetID() == id)
		{
			return mod;
		}
	}

	for (auto mod : m_sfxObjectList)
	{
		if (mod->GetID() == id)
		{
			return mod;
		}
	}

	return NULL;
}

void Managers::ModelManager::RegisterObject(size_t id, Rendering::SceneObject *gameObject)
{
	bool found = false;
	for (auto mod : m_objectList)
	{
		if (mod->GetID() == id)
		{
			mod = gameObject;
			found = true;
			break;
		}
	}

	if (!found)
	{
		gameObject->SetID(id);
		m_objectList.push_back(gameObject);
	}
}

void Managers::ModelManager::RegisterSFXObject(size_t id, Rendering::SceneObject *gameObject)
{
	bool found = false;
	for (auto mod : m_sfxObjectList)
	{
		if (mod->GetID() == id)
		{
			mod = gameObject;
			found = true;
			break;
		}
	}

	if (!found)
	{
		gameObject->SetID(id);
		m_sfxObjectList.push_back(gameObject);
	}
}

void Managers::ModelManager::SetBoundingBoxesVisibile(bool value)
{
	for (auto obj : m_objectList)
	{
		if (obj->GetBoundingBox())
		{
			obj->GetBoundingBox()->SetVisible(value);
		}
	}
}

void Managers::ModelManager::Init()
{
}
