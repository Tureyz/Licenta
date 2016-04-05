#pragma once
#include <string>
#include "../../Dependencies/glm/glm.hpp"

namespace Core
{
	struct WindowProperties
	{
		std::string m_name;
		bool m_isReshapable;
		glm::vec2 m_size;
		glm::vec2 m_pos;

		WindowProperties()
		{
			m_name = "Licenta Test Default";
			m_size = glm::vec2(1024, 768);
			m_pos = glm::vec2(300, 300);
			m_isReshapable = true;
		}

		WindowProperties(std::string name, bool isReshapable, glm::vec2 startPosition = glm::vec2(300, 300), glm::vec2 size = glm::vec2(1024, 768))
		{
			this->m_name = name;
			this->m_isReshapable = isReshapable;
			this->m_size = m_size;
			this->m_pos = m_pos;
		}

		WindowProperties(const WindowProperties& windowInfo)
		{
			m_name = windowInfo.m_name;
			m_pos = windowInfo.m_pos;
			m_size = windowInfo.m_size;
			m_isReshapable = windowInfo.m_isReshapable;
		}

		void operator=(const WindowProperties& windowInfo)
		{

			m_name = windowInfo.m_name;
			m_pos = windowInfo.m_pos;
			m_size = windowInfo.m_size;
			m_isReshapable = windowInfo.m_isReshapable;
		}

	};
}