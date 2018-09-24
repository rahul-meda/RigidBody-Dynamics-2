#pragma once

#include "Model.h"
#include <vector>

namespace Graphics
{
	class Line : public Model
	{
	public:
		Line(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices)
			:Model(vertices, indices), lineWidth(3.0f)
		{
			primType = GL_LINES;
			SetColor(glm::vec3(1.0));
		}

		void SetWidth(const float width)
		{
			lineWidth = width;
		}

		void Render()
		{
			glLineWidth(lineWidth);
			Model::Render();
		}

	private:
		float lineWidth;
	};
}
