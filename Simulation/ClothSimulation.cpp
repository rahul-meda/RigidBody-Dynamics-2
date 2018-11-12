
#include "ClothSimulation.h"
#include "ObjParser.h"
#include "Spring.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "Camera.h"
#include "Geometry.h"
#include "Model.h"
#include "PrimitiveQuery.h"
#include "SphereCollider.h"
#include "HullCollider.h"

ClothSimulation& ClothSimulation::GetInstance()
{
	static ClothSimulation instance;
	return instance;
}

void ClothSimulation::OnInit(GLFWwindow* window)
{
	Simulation::OnInit(window);

	advStep = false;

	HMesh mesh;
	ModelData model;

	float radius = 0.05f;
	CreateSphere(radius, model);
	particleModel = new Model(model.vertices, model.indices);

	const int NC = 21;
	const int NR = 21;
	for (int r = 0; r < NR; r++)
	{
		for (int c = 0; c < NC; c++)
		{
			nodes.push_back(new Particle(0.001f, glm::vec3(0.3f*(float)(c), 0, -0.3*(float)(r + 1))));
		}
	}

	const int ks = 10.0f;	// structural springs
	const int kd = 1.0f;	// shear springs

	// horizontal connections
	for (int r = 0; r < NR; r++)
	{
		for (int c = 0; c < NC-1; c++)
		{
			springs.push_back(new Spring(nodes[c + NC*r], nodes[c + 1 + NC*r], ks));
		}
	}
	// vertical connections
	for (int r = 0; r < NR - 1; r++)
	{
		for (int c = 0; c < NC; c++)
		{
			springs.push_back(new Spring(nodes[c + NC*r], nodes[c + NC + NC*r], ks));
		}
	}
	// diagonal connections L->R
	for (int r = 0; r < NR - 1; r++)
	{
		for (int c = 0; c < NC - 1; c++)
		{
			springs.push_back(new Spring(nodes[c + NC*r], nodes[c + NC + 1 + NC*r], kd));
		}
	}
	// R->L
	for (int r = 0; r < NR - 1; r++)
	{
		for (int c = 1; c < NC; c++)
		{
			springs.push_back(new Spring(nodes[c + NC*r], nodes[c + NC - 1 + NC*r], kd));
		}
	}

	nodes.push_back(new Particle(0, glm::vec3(0)));
	//springs.push_back(new Spring(nodes[0], nodes.back(), ks));
	nodes.push_back(new Particle(0, glm::vec3(0.3f*(float)(NC-1), 0, 0)));
	//springs.push_back(new Spring(nodes[NC-1], nodes.back(), ks));

	vertices.reserve(springs.size() * 2);
	indices.reserve(springs.size() * 2);
	int i = 0;
	for (Spring* s : springs)
	{
		vertices.push_back(s->GetParticleA()->GetPosition());
		vertices.push_back(s->GetParticleB()->GetPosition());
		indices.push_back(i);
		indices.push_back(i+1);
		i += 2;
	}

	Body body;
	Collider* collider;
	/*radius = 1.0f;
	CreateSphere(radius, model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(3.2, -3.2, -3.2));
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetMass(0.0f);
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	collider = new SphereCollider(radius);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);*/

	/*ParseObj("resources/box.obj", mesh);
	mesh.Scale(glm::vec3(1.0,0.1,1.0));
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(3.2, -3.2, -3.2));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetColor(glm::vec3(1.0, 0.9, 0.3));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);*/

	ParseObj("resources/teapot_normalized.obj", mesh);
	mesh.Scale(glm::vec3(3.0f));
	mesh.GetModelData(model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(3.2, -3.2, -3.2));
	body.SetMass(0.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetVelocity(glm::vec3(0));
	body.SetColor(glm::vec3(0.5f));
	body.SetGroup(1);
	bodies.push_back(body);
	std::vector<HMesh> meshes;
	ParseObj("resources/teapot_hulls_normalized.obj", meshes, true);
	for (int i = 0; i < meshes.size(); i++)
	{
		meshes[i].Scale(glm::vec3(3.0f));
		collider = new HullCollider(meshes[i]);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
	}
}

void ClothSimulation::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	glm::vec3 p = nodes[1]->GetPosition();
	int mid = 10 * 21 + 10;

	float t = 0.001f;
	if (keys[GLFW_KEY_DOWN])
		nodes[mid]->SetPosition(p + t*glm::vec3(0, -1.0, 0));
	if (keys[GLFW_KEY_UP])
		nodes[mid]->SetPosition(p + t*glm::vec3(0, 1.0, 0));
	if (keys[GLFW_KEY_RIGHT])
		nodes[mid]->SetPosition(p + t*glm::vec3(1.0, 0, 0));
	if (keys[GLFW_KEY_LEFT])
		nodes[mid]->SetPosition(p + t*glm::vec3(-1.0, 0, 0));
	if (keys[GLFW_KEY_T])
		advStep = true;
}

void ClothSimulation::Step(const float dt)
{
	for (Particle* p : nodes)
		p->Update(dt, RK4);

	// check for collisions
	particleContacts.clear();
	for (Collider* c : colliders)
	{
		for (Particle* p : nodes)
		{
			QueryPoint(particleContacts, c, p);
		}
	}

	// resolve collisions
	for (ParticleContact& c : particleContacts)
	{
		//c.p->SetPosition(c.p->GetPosition() + c.penetration * c.normal);
		//float f = c.p->GetMass()*9.8f*glm::dot(glm::vec3(0, -1, 0), c.normal);
		//c.p->AddForce(f * c.normal);
		//hack
		c.p->SetMass(0);
	}

	for (Spring* s : springs)
		s->Solve();
}

void ClothSimulation::Update()
{
	Simulation::Update();

	const static float dt = 1.0f / 100.0f; //0.002f;

	if (!pauseStep)
	{
		Step(dt);
	}
	else
	{
		if (advStep)
		{
			Step(dt);
			advStep = false;
		}
	}

	static glm::mat4 T(1), M(1);
	glm::mat4 V = Camera::GetInstance().GetViewMatrix();
	glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
	glm::mat4 VP = P * V;
	ModelData model;
	float r, g, b;

	for (Particle* p : nodes)
	{
		r = p->GetPosition().x / 6.3f;
		g = (p->GetPosition().y + 3.0f) / 6.0f;
		b = (p->GetPosition().z  + 6.9f)/ 6.6f;
		T = glm::translate(p->GetPosition());
		particleModel->SetColor(glm::vec3(r, g, b));
		particleModel->SetMVP(VP * T);
		//particleModel->Render();
	}

	int i = 0;
	for (Spring* s : springs)
	{
		vertices[i] = s->GetParticleA()->GetPosition();
		vertices[i+1] = s->GetParticleB()->GetPosition();
		i += 2;
	}

	Model* clothMesh = new Model(vertices, indices);
	clothMesh->SetPrimitive(GL_LINES);
	clothMesh->SetMVP(VP);
	clothMesh->Render();
	delete clothMesh;
}