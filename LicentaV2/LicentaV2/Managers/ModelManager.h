#pragma once
#include <vector>

#include "../Rendering/SceneObject.h"

namespace Managers
{
	class ModelManager
	{
	public:
		ModelManager();
		~ModelManager();

		void Init();
		void Draw(const glm::mat4& viewProjection);
		void FixedUpdate();
		void Update();
		void DeleteAllModels();
		void DeleteModel(std::size_t id);
		const Rendering::SceneObject* GetModel(unsigned long id) const;

		void RegisterObject(size_t id, Rendering::SceneObject *gameObject);

		void SetBoundingBoxesVisibile(bool value);

		std::vector<Rendering::SceneObject*> *GetModelListPtr() { return &m_objectList; }

		float GetDt() const { return m_dt; }
		void SetDt(float val) { m_dt = val; }

	private:

		std::vector<Rendering::SceneObject*> m_objectList;

		float m_dt;
	};
}