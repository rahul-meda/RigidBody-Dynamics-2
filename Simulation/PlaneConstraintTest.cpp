
#include "PlaneConstraintTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

PlaneConstraintTest& PlaneConstraintTest::GetInstance()
{
	static PlaneConstraintTest instance;
	return instance;
}

void PlaneConstraintTest::OnInit(GLFWwindow* window)
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

	Collider* boxCollider = new HullCollider(mesh);
	Body body;
	body.SetPosition(glm::vec3(0, 1.0, 0));
	body.SetMass(1.0f);
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);
	bodies.back().SetModelData(boxModel);

	PlaneConstraint pc(&bodies[0], glm::vec3(0), glm::vec3(0,1.0f,0));
	planeConstraints.push_back(pc);
}

void PlaneConstraintTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	glm::vec3 p = bodies[0].GetPosition();
	glm::quat o = bodies[0].GetOrientation();
	glm::vec3 axis;
	glm::quat dq;
	float t = 0.1f;
	if (keys[GLFW_KEY_DOWN])
		bodies[1].ApplyForce(200.0f*glm::vec3(0, -1.0, 0));
	if (keys[GLFW_KEY_UP])
		bodies[1].ApplyForce(200.0f*glm::vec3(0, 1.0, 0));
	if (keys[GLFW_KEY_RIGHT])
		bodies[1].ApplyForce(200.0f*glm::vec3(1.0, 0, 0));
	if (keys[GLFW_KEY_LEFT])
		bodies[1].ApplyForce(200.0f*glm::vec3(-1.0, 0, 0));
	if (keys[GLFW_KEY_X])
	{
		axis = glm::vec3(1, 0, 0);
		dq = glm::quat(0, axis*t);
		o += dq*o*0.5f;
		o = glm::normalize(o);
		bodies[1].SetOrientation(o);
	}
}