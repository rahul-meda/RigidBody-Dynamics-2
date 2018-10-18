
#include "PositionJointTest.h"
#include "ObjParser.h"
#include "Poly.h"
#include "HullCollider.h"
#include "Collider.h"

PositionJointTest& PositionJointTest::GetInstance()
{
	static PositionJointTest instance;
	return instance;
}

void PositionJointTest::OnInit(GLFWwindow* window)
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

	// ceiling
	Collider* boxCollider = new HullCollider(mesh);
	Body body;
	body.SetPosition(glm::vec3(0, 1.0, 0));
	body.SetMass(0.0f);
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);
	bodies.back().SetModelData(boxModel);

	// links
	float g = 4.0f;	// gap b/w links
	float hg = 2.0f;
	float s = 4.0f;	// y scale of links
	float hs = 2.0f;
	float N = 1;	// num. of links
	float yn = g + s;

	boxCollider = new HullCollider(mesh);
	body.SetPosition(glm::vec3(0, -(g + hs), 0));
	body.SetMass(1.0f);
	bodies.push_back(body);
	bodies.back().AddCollider(boxCollider);
	colliders.push_back(boxCollider);
	bodies.back().SetModelData(boxModel);

	PositionJoint pj(&bodies[0], &bodies[1], glm::vec3(0, -hg, 0));
	posJoints.push_back(pj);

	for (int i = 1; i < 2; i++)
	{
		boxCollider = new HullCollider(mesh);
		body.SetPosition(glm::vec3(0, -(g + hs + yn*(float)i), 0));
		body.SetMass(1.0f);
		bodies.push_back(body);
		bodies.back().AddCollider(boxCollider);
		colliders.push_back(boxCollider);
		bodies.back().SetModelData(boxModel);

		pj = PositionJoint(&bodies[i], &bodies[i+1], glm::vec3(0, -(hg + yn*(float)i), 0));
		posJoints.push_back(pj);
	}
}

void PositionJointTest::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	Simulation::OnKeyInput(window, key, code, action, mods);

	glm::vec3 p = bodies[1].GetPosition();
	glm::quat o = bodies[1].GetOrientation();
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