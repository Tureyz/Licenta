#pragma once
namespace Core {

	struct ContextProperties
	{
		int m_majorVersion, m_minorVersion;
		bool m_core;

		ContextProperties()
		{
			m_majorVersion = 3;
			m_minorVersion = 3;
			m_core = true;
		}

		ContextProperties(int major_version, int minor_version, bool core)
		{
			this->m_majorVersion = major_version;
			this->m_minorVersion = minor_version;
			this->m_core = core;
		}

		ContextProperties(const ContextProperties& other)
		{
			this->m_majorVersion = other.m_majorVersion;
			this->m_minorVersion = other.m_minorVersion;
			this->m_core = other.m_core;
		}

		void operator=(const ContextProperties& rhs)
		{
			this->m_majorVersion = rhs.m_majorVersion;
			this->m_minorVersion = rhs.m_minorVersion;
			this->m_core = rhs.m_core;
		}
		
	};
}