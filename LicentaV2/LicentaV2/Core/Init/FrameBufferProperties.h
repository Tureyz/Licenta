#pragma once
#include "../../Dependencies/glew/glew.h"
#include "../../Dependencies/freeglut/freeglut.h"

namespace Core {

	struct FrameBufferProperties {

		unsigned int m_flags;
		bool m_msaa;//to enable or disable it when wee need it

		FrameBufferProperties()
		{
			//default
			m_flags = GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH;
			m_msaa = false;
		}

		FrameBufferProperties(bool color, bool depth, bool stencil, bool msaa)
		{
			m_flags = GLUT_DOUBLE; //this is a must
			if (color)
				m_flags |= GLUT_RGBA | GLUT_ALPHA;
			if (depth)
				m_flags |= GLUT_DEPTH;
			if (stencil)
				m_flags |= GLUT_STENCIL;
			if (msaa)
				m_flags |= GLUT_MULTISAMPLE;
			this->m_msaa = msaa;
		}

		FrameBufferProperties(const FrameBufferProperties& other)
		{
			this->m_flags = other.m_flags;
			this->m_msaa = other.m_msaa;
		}

		void operator=(const FrameBufferProperties& rhs)
		{
			this->m_flags = rhs.m_flags;
			this->m_msaa = rhs.m_msaa;
		}
	};
}