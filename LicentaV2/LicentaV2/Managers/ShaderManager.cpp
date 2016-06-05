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

std::string ShaderManager::ReadShader(const std::string& filename)
{
	std::string shaderCode;
	std::ifstream shaderFile(filename, std::ios::in);

	if (!shaderFile.good())
	{
		std::cout << "ERROR reading shader file " << filename.c_str() << std::endl;
		std::terminate();
	}

	return std::string((std::istreambuf_iterator<char>(shaderFile)), std::istreambuf_iterator<char>());
}

GLuint Managers::ShaderManager::CreateShader(GLenum shaderType, const std::string& source)
{
	int ret = 0;

	GLuint shader = glCreateShader(shaderType);
	const char *shaderCodePtr = source.c_str();
	const int shaderCodeSize = (const int) source.size();

	glShaderSource(shader, 1, &shaderCodePtr, &shaderCodeSize);
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &ret);

	if (ret == GL_FALSE)
	{		
		std::cout << "ERROR compiling shader" << std::endl;
	}
	return shader;
}

void ShaderManager::CreateProgram(const std::string& vertexShaderFilename, const std::string& fragmentShaderFilename)
{
	std::string vertexShaderCode = ReadShader(vertexShaderFilename);
	std::string fragmentShaderCode = ReadShader(fragmentShaderFilename);

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
		std::cout << "SHADER LINK ERROR" << std::endl;
		return;
	}

	shaderProgram = program;
}

const GLuint ShaderManager::GetShader()
{
	return shaderProgram;
}