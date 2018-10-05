
#include "ParserTest.h"
#include "ObjParser.h"
#include "Poly.h"

ParserTest& ParserTest::GetInstance()
{
	static ParserTest instance;
	return instance;
}

void ParserTest::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ParseObj("resources/box.obj", mesh);

	std::vector<glm::vec3> vertices;
	std::vector<int> indices;

	for (auto vert : mesh.vertices)
	{
		vertices.push_back(vert->position);
	}

	mesh.GetTriangleIndices(indices);

	Body body;
	Model* box = new Poly(vertices, indices);
	body.SetPosition(glm::vec3(0));
	indices.clear();
	mesh.GetLineIndices(indices);

	bodies.push_back(body);
}