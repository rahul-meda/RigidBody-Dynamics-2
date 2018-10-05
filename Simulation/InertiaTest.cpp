
#include "InertiaTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

InertiaTest& InertiaTest::GetInstance()
{
	static InertiaTest instance;
	return instance;
}

void InertiaTest::OnInit(GLFWwindow* window)
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
	Model* boxModel = new Poly(vertices, indices);

	indices.clear();
	mesh.GetLineIndices(indices);
	static_cast<Poly*>(boxModel)->SetFrame(vertices, indices);

	Collider* boxCollider = new HullCollider(mesh);
	boxCollider->SetPosition(glm::vec3(0));
	boxCollider->SetModel(boxModel);
	boxCollider->SetScale(glm::vec3(3.0, 1.0, 2.0));

	Body body;
	body.SetPosition(glm::vec3(0));
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);

	colliders.push_back(boxCollider);
}

void InertiaTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	float w = 1.0f;
	if (keys[GLFW_KEY_R])
		bodies[0].SetAngularVelocity(w*glm::vec3(1, 0, 0));
	if (keys[GLFW_KEY_T])
		bodies[0].SetAngularVelocity(w*glm::vec3(0, 1, 0));
	if (keys[GLFW_KEY_Y])
		bodies[0].SetAngularVelocity(w*glm::vec3(0, 0, 1));

	if (keys[GLFW_KEY_F])
		bodies[0].ApplyForce(bodies[0].GetMass()*3.0f*glm::vec3(0, 1, 0), glm::vec3(3, -1, 2));
}