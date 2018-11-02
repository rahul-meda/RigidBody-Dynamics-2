
#pragma once

#include "Model.h"
#include <vector>

class Poly : public Model
{
public:
	Poly();

	Poly(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices)
		: Model(vertices, indices)
	{
		primType = GL_TRIANGLES;
	}

	~Poly()
	{
		delete frame;
	}

	void SetFrame(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices)
	{
		frame = new Poly(vertices, indices);
		frame->SetPrimitive(GL_LINES);
		frame->SetColor(glm::vec3(0.9));
	}

	Poly* GetFrame() const
	{
		return frame;
	}

private:
	Poly* frame;
};
