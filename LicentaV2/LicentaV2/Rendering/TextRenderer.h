#pragma once
#include "../Core/Utils.hpp"
#include <ft2build.h>
#include FT_FREETYPE_H
#include <vector>
#include <unordered_map>

namespace Rendering
{
	class TextRenderer
	{
	public:
		TextRenderer();

		void PutText(std::string text, glm::vec2 coords);
		void Init();
		void Draw(int width, int height);

		int GetProgram() const { return m_program; }
		void SetProgram(int val) { m_program = val; }
	private:

		FT_Library m_ft;
		FT_Face m_face;
		GLint m_attributeCoord;
		GLint m_uniformTex;
		GLint m_uniformColor;
		GLuint m_vbo, m_vao;

		glm::vec2 m_textSize;

		bool m_initSuccess;

		int m_program;

		std::vector<std::pair<glm::vec2, std::string>> m_textQueue;

		float m_sx;
		float m_sy;

		struct Character {
			GLuint     m_textureID;  // ID handle of the glyph texture
			glm::ivec2 m_size;       // Size of glyph
			glm::ivec2 m_bearing;    // Offset from baseline to left/top of glyph
			GLuint     m_advance;    // Offset to advance to next glyph

			GLuint m_vao;
			GLuint m_vbo;
		};

		std::unordered_map<GLchar, Character> m_characterMap;
	};
}
