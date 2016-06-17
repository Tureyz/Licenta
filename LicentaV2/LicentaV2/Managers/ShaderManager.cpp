#include "ShaderManager.h"

using namespace Managers;

GLuint ShaderManager::shaderProgram;

ShaderManager::ShaderManager(void)
{
}

ShaderManager::~ShaderManager(void)
{
	glDeleteProgram(shaderProgram);
}

std::wstring  ShaderManager::ReadShader(const std::wstring & filename)
{
	std::wstring  shaderCode;
	std::ifstream shaderFile(filename, std::ios::in);

	if (!shaderFile.good())
	{
		std::wcout << "ERROR reading shader file " << filename.c_str() << std::endl;
		std::terminate();
	}

	return std::wstring ((std::istreambuf_iterator<char>(shaderFile)), std::istreambuf_iterator<char>());
}

GLuint Managers::ShaderManager::CreateShader(GLenum shaderType, const std::wstring & source)
{
	int ret = 0;

	GLuint shader = glCreateShader(shaderType);
	std::string src(source.begin(), source.end());
	const char *shaderCodePtr = src.c_str();
	const int shaderCodeSize = (const int) source.size();

	glShaderSource(shader, 1, &shaderCodePtr, &shaderCodeSize);
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &ret);

	if (ret == GL_FALSE)
	{		
		std::wcout << "ERROR compiling shader" << std::endl;
	}
	return shader;
}

void ShaderManager::CreateProgram(const std::wstring & vertexShaderFilename, const std::wstring & fragmentShaderFilename)
{
	std::wstring  vertexShaderCode = ReadShader(vertexShaderFilename);
	std::wstring  fragmentShaderCode = ReadShader(fragmentShaderFilename);

	GLuint vertexShaderHandle = CreateShader(GL_VERTEX_SHADER, vertexShaderCode);
	GLuint fragmentShaderHandle = CreateShader(GL_FRAGMENT_SHADER, fragmentShaderCode);

	int ret = 0;

	GLuint program = glCreateProgram();
	glAttachShader(program, vertexShaderHandle);
	glAttachShader(program, fragmentShaderHandle);

 	glLinkProgram(program);
 	glGetProgramiv(program, GL_LINK_STATUS, &ret);
	if (ret == GL_FALSE)
	{		
		std::wcout << "SHADER LINK ERROR" << std::endl;
		return;
	}

	shaderProgram = program;
}

const GLuint ShaderManager::GetShader()
{
	return shaderProgram;
}