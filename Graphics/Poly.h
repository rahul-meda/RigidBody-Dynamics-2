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

		~Poly()
		{
			delete frame;
		}

		void SetFrame()
		{
			frame = new Poly(vertices, indices);
			frame->primType = GL_LINES;
			frame->SetColor(glm::vec3(0.9));
		}

		Poly* GetFrame() const
		{
			return frame;
		}

	private:
		// to draw a frame around the polyhedron (give it a 3D effect without ligting or shadows)
		Poly* frame;
	};
}
