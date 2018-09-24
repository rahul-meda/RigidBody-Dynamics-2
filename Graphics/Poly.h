#pragma once

#include "Model.h"
#include <vector>

namespace Graphics
{
	class Poly : public Model
	{
	public:
		Poly(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices)
			:Model(vertices, indices)
		{
			primType = GL_TRIANGLES;
		}
	};
}
