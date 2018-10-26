
#include "FractureTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

FractureTest& FractureTest::GetInstance()
{
	static FractureTest instance;
	return instance;
}

void FractureTest::OnInit(GLFWwindow* window)
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
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	ParseObj("resources/box.obj", mesh);
	mesh.GetModelData(model);
	collider = new HullCollider(mesh);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(10.0, 0.0, 0.0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.78f, glm::vec3(0, 0, 1)));
	body.SetVelocity(glm::vec3(-2.0f,0,0));
	body.SetColor(glm::vec3(0.4, 0.9, 0.1));
	bodies.push_back(body);
	bodies.back().AddCollider(collider);
	colliders.push_back(collider);

	ParseObj("resources/teapot.obj", mesh);
	mesh.GetModelData(model);
	body.SetModelData(model);
	body.SetPosition(glm::vec3(0.0, 0.0, 0.0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0, 0, 1)));
	body.SetVelocity(glm::vec3(0));
	body.SetColor(glm::vec3(0.4, 0.9, 0.1));
	body.SetTag("teapot");
	bodies.push_back(body);
	std::vector<HMesh> meshes;
	ParseObj("resources/teapot_hulls.obj", meshes, true);
	for (int i = 0; i < meshes.size(); i++)
	{
		collider = new HullCollider(meshes[i]);
		bodies.back().AddCollider(collider);
		colliders.push_back(collider);
	}
}

void FractureTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
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
		bodies[1].SetAngularVelocity(glm::vec3(1.0, 0, 0));
	}
	if (keys[GLFW_KEY_C])
	{
		bodies[1].SetAngularVelocity(glm::vec3(0, 1.0, 0));
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
		bodies[1].ApplyForce(200.0f*(glm::vec3(0, 1.0, 0)), glm::vec3(5.0, 0, 0));
	}
}

void FractureTest::Update()
{
	Simulation::Update();

	static bool shatter = false;
	if (!shatter)
	{
		for (auto m : manifolds)
		{
			for (auto c : m.contacts)
			{
				if (c.GetBodyA()->GetTag() == "teapot" || c.GetBodyB()->GetTag() == "teapot")
					shatter = true;
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
			bodies.push_back(body);
			bodies.back().AddCollider(c);
		}

		shattered = true;
	}
}