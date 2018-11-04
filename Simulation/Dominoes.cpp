
#include "Dominoes.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"
#include "SphereCollider.h"

Dominoes& Dominoes::GetInstance()
{
	static Dominoes instance;
	return instance;
}

void Dominoes::OnInit(GLFWwindow* window)
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
	body.SetPosition(glm::vec3(0, -1.0, 0));
	body.SetMass(0.0f);
	body.SetColor(glm::vec3(0.7, 0.7, 0.6));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	int N = 3;
	glm::vec3 s(0.2f, 0.5f, 0.05f);
	float g = 0.5f;
	for (int i = 0; i < N; i++)
	{
		ParseObj("resources/box.obj", mesh);
		mesh.Scale(s);
		mesh.GetModelData(model);
		collider = new HullCollider(mesh);
		body.SetModelData(model);
		body.SetPosition(glm::vec3(0, s.y, -g*(float)i));
		body.SetMass(1.0f);
		body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
		//body.SetFriction(0.7f);
		//body.SetRestitution(0.9f);
		body.SetColor(glm::vec3(0.9, 0.6, 0.4));
		bodies.push_back(body);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
	}

	float radius = 0.1f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.1, 1.0, 1.0));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetVelocity(glm::vec3(0,0,-3.0));
	body.SetMass(1.0f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	/*radius = 0.1f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0, 1.0, -1.0));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetVelocity(glm::vec3(7.0, 0, 0));
	body.SetMass(1.0f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);*/
}

void Dominoes::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
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