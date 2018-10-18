
#include "CompositeBodyTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

CompositeBodyTest& CompositeBodyTest::GetInstance()
{
	static CompositeBodyTest instance;
	return instance;
}

void CompositeBodyTest::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ModelData model;
	Collider* collider;
	Body body;
	int id = 0;

	// floor
	ParseObj("resources/floor.obj", mesh);
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0, -10.0, 0));
	body.SetMass(0.0f);
	body.SetColor(glm::vec3(0.7, 0.7, 0.6));
	body.SetID(++id);
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	ParseObj("resources/teapot.obj", mesh);
	mesh.GetModelData(model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.0, 0.0, 0.0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.78f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.4, 0.9, 0.1));
	body.SetID(++id);
	bodies.push_back(body);
	std::vector<HMesh> meshes;
	ParseObj("resources/teapot_hulls.obj", meshes);
	for (int i = 0; i < meshes.size(); i++)
	{
		collider = new HullCollider(meshes[i]);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
	}
}

void CompositeBodyTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
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
		bodies[1].SetPosition(p + t*glm::vec3(1.0, 0.0, 0));
	if (keys[GLFW_KEY_LEFT])
		bodies[1].SetPosition(p + t*glm::vec3(-1.0, 0, 0));
	if (keys[GLFW_KEY_Y])
	{
		axis = glm::vec3(0, 0, -1);
		dq = glm::quat(0, axis*t);
		o += dq*o*0.5f;
		o = glm::normalize(o);
		bodies[1].SetOrientation(o);
	}
	if (keys[GLFW_KEY_X])
	{
		bodies[1].SetAngularVelocity(glm::vec3(1.0,0,0));
	}
	if (keys[GLFW_KEY_C])
	{
		bodies[1].SetAngularVelocity(glm::vec3(0,1.0,0));
	}
	if (keys[GLFW_KEY_V])
	{
		bodies[1].SetAngularVelocity(glm::vec3(0, 0, 1.0));
	}
	if (keys[GLFW_KEY_B])
	{
		bodies[1].SetAngularVelocity(glm::vec3(0));
	}
	if (keys[GLFW_KEY_F])
	{
		bodies[1].ApplyForce(200.0f*(glm::vec3(0,1.0,0)), glm::vec3(5.0,0,0));
	}
}