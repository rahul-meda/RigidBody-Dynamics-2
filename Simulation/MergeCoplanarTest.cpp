
#include "MergeCoplanarTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

MergeCoplanarTest& MergeCoplanarTest::GetInstance()
{
	static MergeCoplanarTest instance;
	return instance;
}

void MergeCoplanarTest::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ModelData model;
	Collider* collider;
	Body body;

	// floor
	ParseObj("resources/floor.obj", mesh);
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0, -10.0, 0));
	body.SetMass(0.0f);
	body.SetColor(glm::vec3(0.7, 0.7, 0.6));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	ParseObj("resources/coplanar_1.obj", mesh, true);
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.0, 0.0, 0.0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.78f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.4, 0.9, 0.1));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
}

void MergeCoplanarTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
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