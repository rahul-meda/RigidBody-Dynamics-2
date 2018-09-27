
#include "ParserTest.h"
#include "ObjParser.h"
#include "Poly.h"

namespace Simulation
{
	ParserTest& ParserTest::GetInstance()
	{
		static ParserTest instance;
		return instance;
	}

	void ParserTest::OnInit(GLFWwindow* window)
	{
		Simulation::OnInit(window);

		Geometry::HMesh mesh;
		Geometry::ParseObj("resources/box.obj", mesh);

		std::vector<glm::vec3> vertices;
		std::vector<int> indices;

		for each (auto vert in mesh.vertices)
		{
			vertices.push_back(vert->position);
		}

		mesh.GetTriangleIndices(indices);

		Physics::Body body;
		Graphics::Model* box = new Graphics::Poly(vertices, indices);
		body.SetModel(box);
		body.SetPosition(glm::vec3(0));
		indices.clear();
		mesh.GetLineIndices(indices);
		body.GetModel()->SetFrame(vertices, indices);

		bodies.push_back(body);

		int x = 1;
	}
}