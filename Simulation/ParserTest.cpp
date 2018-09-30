
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

		for (auto vert : mesh.vertices)
		{
			vertices.push_back(vert->position);
		}

		mesh.GetTriangleIndices(indices);

		Physics::Body body;
		Graphics::Model* box = new Graphics::Poly(vertices, indices);
		body.SetPosition(glm::vec3(0));
		indices.clear();
		mesh.GetLineIndices(indices);

		bodies.push_back(body);
	}
}