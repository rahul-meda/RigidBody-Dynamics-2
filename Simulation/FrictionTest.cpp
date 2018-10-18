
#include "FrictionTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

FrictionTest& FrictionTest::GetInstance()
{
	static FrictionTest instance;
	return instance;
}

void FrictionTest::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ParseObj("resources/box.obj", mesh);

	std::vector<glm::vec3> vertices;
	std::vector<int> indices;
	std::vector<int> frameIndices;

	for (auto vert : mesh.vertices)
	{
		vertices.push_back(vert->position);
	}
	mesh.GetTriangleIndices(indices);
	mesh.GetLineIndices(frameIndices);
	
	ModelData boxModel(vertices, indices, frameIndices);

	// floor
	Collider* boxCollider = new HullCollider(mesh);
	Body body;
	body.SetPosition(glm::vec3(0));
	body.SetMass(0.0f);
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);

	boxCollider = new HullCollider(mesh);
	body.SetPosition(glm::vec3(0.0, 3.0, 0.0));
	body.SetVelocity(glm::vec3(5.0,0.0,0.0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::quat(0.78, 0, 1, 0));
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);

	boxCollider = new HullCollider(mesh);
	body.SetPosition(glm::vec3(0.0, 3.0, 5.0));
	body.SetVelocity(glm::vec3(7.0,0.0,0.0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::quat(0.78*2.0, 0, 1, 0));
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);
}

void FrictionTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	glm::vec3 p = bodies[1].GetPosition();
	glm::quat o = bodies[1].GetOrientation();
	glm::vec3 axis;
	glm::quat dq;
	float t = 0.1f;
	if (keys[GLFW_KEY_DOWN])
		bodies[1].SetPosition(p + t*glm::vec3(0, -1.0, 0));
	if (keys[GLFW_KEY_UP])
		bodies[1].SetPosition(p + t*glm::vec3(0, 1.0, 0));
	if (keys[GLFW_KEY_RIGHT])
		bodies[1].SetPosition(p + t*glm::vec3(1.0, 0, 0));
	if (keys[GLFW_KEY_LEFT])
		bodies[1].SetPosition(p + t*glm::vec3(-1.0, 0, 0));
	if (keys[GLFW_KEY_X])
	{
		axis = glm::vec3(1, 0, 0);
		dq = glm::quat(0, axis*t);
		o += dq*o*0.5f;
		o = glm::normalize(o);
		bodies[1].SetOrientation(o);
	}
}