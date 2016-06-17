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

		void CreateProgram(const std::wstring & VertexShaderFilename,	const std::wstring & FragmentShaderFilename);

		static const GLuint GetShader();

	private:

		std::wstring  ReadShader(const std::wstring & filename);
		GLuint CreateShader(GLenum shaderType,	const std::wstring & source);

		static GLuint shaderProgram;
	};
}
