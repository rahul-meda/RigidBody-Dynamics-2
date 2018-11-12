
#include "LemonadeMachine.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BroadPhase.h"

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
	RevoluteJoint rj(&bodies[5], &bodies[0], glm::vec3(1.0f, 0.5f, -0.3f), glm::vec3(1.0f,0,0));
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
	body.SetRestitution(0.8f);
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
	body.SetPosition(glm::vec3(-4.25f, -0.6f - 0.025, -0.15));
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
	body.SetPosition(glm::vec3(-4.53f, -0.6f - 0.05, -0.9f));
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
	body.SetPosition(glm::vec3(-4.25f, -0.6f - 0.075, -1.65f));
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
	body.SetPosition(glm::vec3(-3.4f, -0.6f - 0.1, -2.0f));
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
	body.SetPosition(glm::vec3(-2.6, -0.95f-0.025, -2.0f));
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
	radius = 0.1f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -1.3, -1.25));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetMass(1.0f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//29
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.2f, 0.05f, 0.1f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -1.45, -1.3));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//30
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.2f, 0.05f, 1.0f));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -2.0, -2.1));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(-0.5f, glm::vec3(1.0, 0, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//31
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.2f, 0.05f, 0.25));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -2.9, -3.05));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(1.0, 0, 0)));
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//32-40
	int N = 9;
	for (int i = 0; i < N; i++)
	{
		ParseObj("resources/box.obj", mesh);
		mesh.Scale(glm::vec3(0.2f, 0.5f, 0.05f));
		mesh.GetModelData(model);
		collider = new HullCollider(mesh);
		body.SetModelData(model);
		body.SetPosition(glm::vec3(-5.0, -3.2, -3.7 - 0.5*(float)(i)));
		body.SetMass(1.0f);
		body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
		body.SetRestitution(0.3f);
		body.SetColor(glm::vec3(0.9, 0.6, 0.4));
		bodies.push_back(body);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
	}
	//41
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(2.0f, 0.2f, 2.5));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -3.9, -6.0));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetRestitution(0.3f);
	body.SetColor(glm::vec3(0.8, 0.8, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//42
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.1));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -2.7, -8.5));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetColor(glm::vec3(0.8, 0.8, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//43
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.1));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -3.0, -8.5));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetCanSleep(false);
	body.SetColor(glm::vec3(0.9, 0.6, 0.4));
	body.SetGroup(2);
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	//44-52
	N = 9;
	for (int i = 0; i < N; i++)
	{
		ParseObj("resources/box.obj", mesh);
		mesh.Scale(glm::vec3(0.1));
		mesh.GetModelData(model);
		collider = new HullCollider(mesh);
		body.SetModelData(model);
		body.SetPosition(glm::vec3(-5.2, -3.0, -8.8 - 0.3*(float)(i)));
		body.SetMass(1.0f);
		body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
		body.SetCanSleep(false);
		body.SetColor(glm::vec3(0.9, 0.6, 0.4));
		body.SetGroup(0);
		bodies.push_back(body);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
		PositionJoint pj(&bodies[43 + i], &bodies[43 + i + 1], glm::vec3(-5.2, -3.0, -8.8 - 0.3*(float)(i)+0.15));
		posJoints.push_back(pj);
	}
	//53
	ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(0.1));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -3.0, -8.8 - 0.3*(float)(N) - 0.1));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1.0)));
	body.SetCanSleep(true);
	body.SetColor(glm::vec3(0.8, 0.8, 0.4));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);
	PositionJoint pj(&bodies[43 + N], &bodies[43 + N + 1], glm::vec3(-5.2, -3.0, -8.8 - 0.3*(float)(N)+0.05));
	posJoints.push_back(pj);
	pj = PositionJoint(&bodies[43], &bodies[42], glm::vec3(-5.2, -2.85, -8.5));
	posJoints.push_back(pj);
	//54
	ParseObj("resources/teapot_normalized.obj", mesh);
	mesh.GetModelData(model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(-5.2, -5.5, -8.8 - 0.3*(float)(N)-0.1-1.5));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetCanSleep(true);
	body.SetColor(glm::vec3(0.4, 0.9, 0.1));
	body.SetGroup(1);
	bodies.push_back(body);
	std::vector<HMesh> meshes;
	ParseObj("resources/teapot_hulls_normalized.obj", meshes, true);
	for (int i = 0; i < meshes.size(); i++)
	{
		collider = new HullCollider(meshes[i]);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
	}
	rj = RevoluteJoint(&bodies[54], &bodies[0], glm::vec3(-5.2, -5.5, -8.8 - 0.3*(float)(N)-0.1 - 1.5), glm::vec3(0, 1.0, 0));
	revJoints.push_back(rj);

	BroadPhase::GetInstance().Init(colliders);
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
	if (keys[GLFW_KEY_F])
	{
		glm::vec3 F = posJoints[3].GetReactionForce();
		float M = glm::length2(F);
		if (M > 1.0f)
		{
			int x = 1;
		}
		std::cout << M << std::endl;
	}
}

void LemonadeMachine::Update()
{
	Simulation::Update();

	static bool done = false;
	if (!done)
	{
		for (auto m : manifolds)
		{
			for (auto c : m.contacts)
			{
				if (c.GetBodyA()->GetGroup() == 2 || c.GetBodyB()->GetGroup() == 2)
				{
					posJoints.pop_back();
					done = true;
					std::cout << "done" << std::endl;
					break;
				}
			}
		}
	}

	static bool shatter = false;
	if (!shatter)
	{
		for (auto m : manifolds)
		{
			for (auto c : m.contacts)
			{
				if (c.GetBodyA()->GetGroup() == 1 || c.GetBodyB()->GetGroup() == 1)
				{
					if (c.GetBodyA()->GetInvMass() != 0 && c.GetBodyB()->GetInvMass() != 0)
					{
						shatter = true;
						revJoints.pop_back();
						break;
					}
				}
			}
		}
	}

	static bool shattered = false;
	if (shatter && !shattered)
	{
		Body b = bodies.back();
		bodies.pop_back();

		Body body;
		glm::vec3 position(0);
		glm::quat orientation;
		ModelData model;

		for (Collider* c : b.GetColliders())
		{
			for (auto v : static_cast<HullCollider*>(c)->GetVertices())
			{
				position += v->position;
			}
			position /= static_cast<HullCollider*>(c)->GetVertexCount();

			for (auto v : static_cast<HullCollider*>(c)->GetVertices())
			{
				v->position -= position;
			}

			static_cast<HullCollider*>(c)->GetModelData(model);
			body.SetModelData(model);
			position = b.LocalToGlobalPoint(position);
			body.SetPosition(position);
			body.SetOrientation(b.GetOrientation());
			body.SetGroup(1);
			bodies.push_back(body);
			bodies.back().AddCollider(c);
		}

		shattered = true;

		BroadPhase::GetInstance().Init(colliders);
	}
}