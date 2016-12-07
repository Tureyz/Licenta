#include "TextRenderer.h"
#include <iostream>
#include "../Dependencies/freeglut/freeglut.h"
#include "../Dependencies/glm/gtc/type_ptr.hpp"
#include "../Dependencies/glm/gtc/matrix_transform.hpp"


Rendering::TextRenderer::TextRenderer()
{
	m_initSuccess = false;
	m_sx = 2.0 / glutGet(GLUT_WINDOW_WIDTH);
	m_sy = 2.0 / glutGet(GLUT_WINDOW_HEIGHT);
}

void Rendering::TextRenderer::PutText(std::string text, glm::vec2 coords)
{
	m_textQueue.push_back(std::make_pair(coords, text));
}

void Rendering::TextRenderer::Init()
{
	int ret = 0;

	ret = FT_Init_FreeType(&m_ft);
	if (ret)
	{
		std::wcerr << L"Could not init freetype library " << ret << std::endl;
		return;
	}

	ret = FT_New_Face(m_ft, "Fonts\\FreeSans.ttf", 0, &m_face);
	if (ret)
	{
		std::wcerr << L"Could not open font FreeSans.ttf " << ret << std::endl;
		return;
	}

	FT_Set_Pixel_Sizes(m_face, 0, 48);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // Disable byte-alignment restriction

	for (GLubyte c = 0; c < 128; c++)
	{
		// Load character glyph 
		if (FT_Load_Char(m_face, c, FT_LOAD_RENDER))
		{
			std::wcout << L"ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
			continue;
		}
		// Generate texture
		GLuint texture;
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_RED,
			m_face->glyph->bitmap.width,
			m_face->glyph->bitmap.rows,
			0,
			GL_RED,
			GL_UNSIGNED_BYTE,
			m_face->glyph->bitmap.buffer
		);
		// Set texture options
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);



		// Now store character for later use
		Character character = {
			texture,
			glm::ivec2(m_face->glyph->bitmap.width, m_face->glyph->bitmap.rows),
			glm::ivec2(m_face->glyph->bitmap_left, m_face->glyph->bitmap_top),
			m_face->glyph->advance.x
		};

		glGenVertexArrays(1, &character.m_vao);
		glBindVertexArray(character.m_vao);

		glGenBuffers(1, &character.m_vbo);
		glBindBuffer(GL_ARRAY_BUFFER, character.m_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_STATIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_characterMap.insert(std::pair<GLchar, Character>(c, character));
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	FT_Done_Face(m_face);
	FT_Done_FreeType(m_ft);



	m_initSuccess = true;
}

void Rendering::TextRenderer::Draw(int width, int height)
{
	if (!m_initSuccess)
		return;
	
	glm::vec3 color(0.f, 0.8f, 0.f);
	GLfloat scale = 1.f;

	glm::mat4 projection = glm::ortho(0.0f, static_cast<GLfloat>(width), 0.0f, static_cast<GLfloat>(height));
	glUniformMatrix4fv(glGetUniformLocation(m_program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

	for (auto line : m_textQueue)
	{
		glm::vec2 pos = line.first;
		std::string text = line.second;

		glUniform3f(glGetUniformLocation(m_program, "textColor"), color.x, color.y, color.z);
		glActiveTexture(GL_TEXTURE0);

		// Iterate through all characters
		std::string::const_iterator c;
		for (c = text.begin(); c != text.end(); c++)
		{
			Character ch = m_characterMap[*c];

			glBindVertexArray(ch.m_vao);
			GLfloat xpos = pos.x + ch.m_bearing.x * scale;
			GLfloat ypos = pos.y - (ch.m_size.y - ch.m_bearing.y) * scale;

			GLfloat w = ch.m_size.x * scale;
			GLfloat h = ch.m_size.y * scale;
			// Update VBO for each character
			GLfloat vertices[6][4] = {
				{ xpos,     ypos + h,   0.0, 0.0 },
				{ xpos,     ypos,       0.0, 1.0 },
				{ xpos + w, ypos,       1.0, 1.0 },

				{ xpos,     ypos + h,   0.0, 0.0 },
				{ xpos + w, ypos,       1.0, 1.0 },
				{ xpos + w, ypos + h,   1.0, 0.0 }
			};
			// Render glyph texture over quad
			glBindTexture(GL_TEXTURE_2D, ch.m_textureID);
			// Update content of VBO memory
			glBindBuffer(GL_ARRAY_BUFFER, ch.m_vbo);
			glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
			// Render quad
			glDrawArrays(GL_TRIANGLES, 0, 6);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			// Now advance cursors for next glyph (note that advance is number of 1/64 pixels)
			pos.x += (ch.m_advance >> 6) * scale; // Bitshift by 6 to get value in pixels (2^6 = 64)
		}
		glBindVertexArray(0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}


	m_textQueue.clear();
}
