
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
	boxCollider->SetPosition(glm::vec3(glm::vec3(0,-10.0,0)));
	boxCollider->SetModelData(boxModel);
	boxCollider->SetScale(glm::vec3(20.0, 1.0, 20.0));
	boxCollider->SetColor(glm::vec3(0.7, 0.7, 0.6));
	Body body;
	body.SetPosition(glm::vec3(0, -10.0, 0));
	body.SetMass(0.0f);
	//body.SetOrientation(glm::angleAxis(-glm::pi<float>() / 4.0f, glm::vec3(0, 0, 1.0)));
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);
	bodies.back().SetColor(glm::vec3(0.7, 0.7, 0.6));
	bodies.back().SetModelData();

	boxCollider = new HullCollider(mesh);
	boxCollider->SetPosition(glm::vec3(5.0,0,0.0));
	boxCollider->SetModelData(boxModel);
	boxCollider->SetScale(glm::vec3(1.0, 1.0, 1.0));
	body.SetPosition(glm::vec3(0.0,0.0,0.0));
	//body.SetVelocity(glm::vec3(0,-1.0,0));
	body.SetMass(1.0f);
	body.SetOrientation(glm::angleAxis(0.0f, glm::vec3(0,0,1.0)));
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);

	boxCollider = new HullCollider(mesh);
	boxCollider->SetPosition(glm::vec3(-5.0,0,0));
	boxCollider->SetModelData(boxModel);
	boxCollider->SetScale(glm::vec3(1.0, 1.0, 1.0));
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);
	bodies.back().SetModelData();
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