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

		void CreateProgram(const std::string& VertexShaderFilename,	const std::string& FragmentShaderFilename);

		static const GLuint GetShader();

	private:

		std::string ReadShader(const std::string& filename);
		GLuint CreateShader(GLenum shaderType,	const std::string& source);

		static GLuint shaderProgram;
	};
}
