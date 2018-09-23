
#pragma once

#include "Shader.h"
#include <glm/glm.hpp>
#include <vector>

namespace Graphics
{
	class Model
	{
	public:
		Model(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices);

		~Model();

		void Render();

		void FillVertexBuffer(GLfloat* pBuffer);

		void FillIndexBuffer(GLuint* pBuffer);

		void SetColor(const glm::vec3 color);

		void SetMVP(const glm::mat4& MVP);

		glm::mat4 GetMVP() const;

	protected:
		GLenum primType;

	private:
		std::vector<glm::vec3> vertices;

		std::vector<int> indices;

		GLuint vaoID;

		GLuint vboVerticesID;

		GLuint vboIndicesID;

		Shader shader;

		glm::vec3 color;

		glm::mat4 MVP;
	};
}