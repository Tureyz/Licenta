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

		void CreatePrograms();

		static const GLuint GetSceneShader();
		static const GLuint GetTextShader();

	private:
		int CreateProgram(const std::wstring & VertexShaderFilename,const std::wstring & FragmentShaderFilename);

		std::wstring  ReadShader(const std::wstring & filename);
		GLuint CreateShader(GLenum shaderType,	const std::wstring & source);

		static GLuint m_textShaderProgram;
		static GLuint m_sceneShaderProgram;
	};
}
