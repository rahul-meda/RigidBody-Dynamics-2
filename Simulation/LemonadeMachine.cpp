
#include "LemonadeMachine.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"
#include "SphereCollider.h"

LemonadeMachine& LemonadeMachine::GetInstance()
{
	static LemonadeMachine instance;
	return instance;
}

void LemonadeMachine::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	HMesh mesh;
	ModelData model;
	Collider* collider;
	Body body;

	//0
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(1.5f,0.2f,1.5f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.0, 0.0, 0.0));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.8, 0.8, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//1
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.5f, 0.3f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.0, 0.7f, -0.8f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//2
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.1f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-0.3f, 1.3f, -0.8f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//3
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.4f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-0.1f, 1.5f, -0.8f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//4
	float radius = 0.1f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-0.3f, 1.7f, -0.8f));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetMass(1.0f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//5
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.25f, 0.05f, 0.7f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(1.0f, 0.5f, -0.3f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	RevoluteJoint rj(&bodies[5], &bodies[0], glm::vec3(1.0f, 0.3f, -0.3f), glm::vec3(1.0f,0,0));
	revJoints.push_back(rj);
	//6
	radius = 0.1f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(1.4f, 0.5f, 0.2f));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetMass(1.0f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//7
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(1.3f, 0.25f, 0.2f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.4f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//8
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.1f, 0.5f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.0f, 0.75f, 0.2f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	rj = RevoluteJoint(&bodies[8], &bodies[0], glm::vec3(0.0f, 0.2f, 0.2f), glm::vec3(0,0,1.0f));
	revJoints.push_back(rj);
	//9
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-1.2f, 0.7f, 0.2f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//10
	radius = 0.1f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-0.8f, 0.9f, 0.2f));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetMass(1.0f);
	body.SetRestitution(0.7f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//11
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(1.0f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-2.3f, 0.1f, 0.2f));
	body.SetMass(0.0f);
	body.SetRestitution(0.3f);
	body.SetOrientation(glm::angleAxis(0.6f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//12
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.6f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-3.4f, -0.6f, 0.2f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//13
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.25f, -0.6f, -0.15));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(-0.78f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//14
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.2f, 0.1f, 0.5f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.53f, -0.6f, -0.9f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//15
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.2f, 0.1f, 0.6f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.25f, -0.6f, -1.65f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(-0.78f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//16
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-3.4f, -0.6f, -2.0f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//17
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-2.6, -0.95f, -2.0f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(-0.6f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//18
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.2f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-1.95f, -1.2f, -2.0f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//19
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.3f, 0.05f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.4f, -0.6f, 0));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(-0.78f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//20
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.05f, 0.3f, 0.5f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.8f, -0.6f, -0.9f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//21
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.05f, 0.3f, 0.5f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.4f, -0.6f, -1.8f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(-0.78f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//22
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.6f, 0.3f, 0.05f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-3.4f, -0.6f, -2.25f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 1, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//23
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.05f, 0.2f, 0.7f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-1.35, -1.0f, -1.5f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetRestitution(0.9f);
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	rj = RevoluteJoint(&bodies[23], &bodies[0], glm::vec3(-1.35, -1.0f, -1.5f), glm::vec3(0, 1.0, 0));
	revJoints.push_back(rj);
	//24
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.05f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-2.3, -1.0f, -1.4f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetDensity(0.5f);
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	rj = RevoluteJoint(&bodies[24], &bodies[0], glm::vec3(-2.3, -1.0f, -1.4f), glm::vec3(0, 1.0, 0));
	revJoints.push_back(rj);
	//25
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.05f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-3.1, -1.1f, -1.1f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	rj = RevoluteJoint(&bodies[25], &bodies[0], glm::vec3(-3.1, -1.1f, -1.1f), glm::vec3(0, 1.0, 0));
	revJoints.push_back(rj);
	//26
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.05f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-3.9, -1.2f, -1.4f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	rj = RevoluteJoint(&bodies[26], &bodies[0], glm::vec3(-3.9, -1.2f, -1.4f), glm::vec3(0, 1.0, 0));
	revJoints.push_back(rj);
	//27
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.5f, 0.1f, 0.05f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-4.7, -1.3f, -1.1f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	rj = RevoluteJoint(&bodies[27], &bodies[0], glm::vec3(-4.7, -1.3f, -1.1f), glm::vec3(0, 1.0, 0));
	revJoints.push_back(rj);
	//28
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(3.0f, 0.2f, 1.5f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-6.5, -2.7, -1.5f));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetDensity(1.0f);
	body.SetRestitution(0.3f);
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.2f, 0.5f, 0.1f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-6.5, -2.0, -1.5f));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
}

void LemonadeMachine::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
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