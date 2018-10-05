
#include "ScaleTest.h"
#include "ObjParser.h"
#include "Poly.h"

ScaleTest& ScaleTest::GetInstance()
{
	static ScaleTest instance;
	return instance;
}

void ScaleTest::OnInit(GLFWwindow* window)
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
	body.SetVelocity(glm::vec3(1.0, 0, 0));
	body.SetMass(1.0);
	indices.clear();
	mesh.GetLineIndices(indices);

	bodies.push_back(body);

	body.SetPosition(glm::vec3(0, -2.0, 0));
	body.SetMass(0.0);

	bodies.push_back(body);
}