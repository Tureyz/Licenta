#pragma once
#include <string>
#include "../Dependencies/glm/glm.hpp"


namespace ScriptLoader
{
	glm::vec3 GetVec3(const std::string scriptPath, const std::string functionName);
}