#pragma once
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include "../Dependencies/glew/glew.h"
#include "../Dependencies/freeglut/freeglut.h"

namespace Managers
{

	class ShaderManager
	{

	public:

		ShaderManager(void);
		~ShaderManager(void);

		//modify char* to std::string
		void CreateProgram(const std::string& shaderName,
			const std::string& VertexShaderFilename,
			const std::string& FragmentShaderFilename);

		static const GLuint GetShader(const std::string&);

	private:
		//modify char* to std::string
		std::string ReadShader(const std::string& filename);
		//modify char* to std::string
		GLuint CreateShader(GLenum shaderType,
			const std::string& source,
			const std::string& shaderName);

		static std::map<std::string, GLuint> programs;
	};
}
