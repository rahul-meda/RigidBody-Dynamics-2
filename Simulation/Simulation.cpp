
#include "Simulation.h"
#include "Poly.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Camera.h"
#include <memory>
#include "NarrowPhase.h"
#include "Mesh.h"
#include "ObjParser.h"
#include "PrimitiveQuery.h"

#define MOUSE_SENSITIVITY 0.1
#define FOV 45.0
#define MAX_BODIES 1000
#define VELOCITY_ITERS 7
#define POSITION_ITERS 3

Simulation::Simulation()
	:debugDraw(false), firstMouseCB(false), pauseStep(true), advanceStep(false), picked(false)
{
	panLeft = panRight = panBot = panTop = false;
}

void Simulation::OnInit(GLFWwindow* window)
{
	glfwGetWindowSize(window, &width, &height);

	mouseX = width / 2.0f;
	mouseY = height / 2.0f;

	glClearColor(0.3, 0.3, 0.3, 0);

	// draw the pixel only if the object is closer to the viewer
	glEnable(GL_DEPTH_TEST); // enables depth-testing
	glDepthFunc(GL_LESS);    // interpret smaller values as closer

	//glViewport(0, 0, width, height);
	Camera::GetInstance().SetProjection(45.0, (float)width / (float)height);
	Camera::GetInstance().SetPosition(glm::vec3(0, 1, 0));

	HMesh mesh;
	ParseObj("resources/box.obj", mesh);
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;

	for (auto vert : mesh.vertices)
	{
		vertices.push_back(vert->position);
	}
	mesh.GetTriangleIndices(indices);
	boxModel = new Poly(vertices, indices);
	boxModel->SetColor(glm::vec3(1.0, 0, 0));

	// reserving space for global data
	// otherwise any pointers or references to container elements will get invalidated on using push_back
	// because push_back reallocates memory, and old memory locations are invalidated
	bodies.reserve(5000);
	colliders.reserve(5000);
	manifolds.reserve(5000);
}

void Simulation::OnWindowResize(GLFWwindow* window, int width, int height)
{
	this->width = width;
	this->height = height;
	glViewport(0, 0, width, height);

	Camera::GetInstance().SetProjection(45.0, (float)width / height);
}

void Simulation::OnMouseMove(GLFWwindow* window, double x, double y)
{
	if (firstMouseCB)
	{
		mouseX = x;
		mouseY = y;
		firstMouseCB = false;
	}

	float dx = mouseX - x;
	float dy = mouseY - y;

	mouseX = x;
	mouseY = y;

	dx *= MOUSE_SENSITIVITY;
	dy *= MOUSE_SENSITIVITY;

	yaw += dx;
	pitch += dy;

	if (mouseX < 20) panLeft = true;
	else panLeft = false;
	if (mouseX > width - 20) panRight = true;
	else panRight = false;
	if (mouseY < 20) panBot = true;
	else panBot = false;
	if (mouseY > height - 20) panTop = true;
	else panTop = false;

	//if (pitch > 89.0f) pitch = 89.0f;
	//if (pitch < -89.0f) pitch = -89.0f;

	Camera::GetInstance().Rotate(yaw, pitch, 0);

	if (picked)
	{
		// read pixel depth at mouse click position - gives screen space 3D point
		float mouseZ = 0;
		glReadPixels(mouseX, height - mouseY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &mouseZ);

		// convert the point from screen space to world space
		glm::vec3 sMouse = glm::vec3(mouseX, height - mouseY, mouseZ);
		glm::mat4 MV = Camera::GetInstance().GetViewMatrix();
		glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
		glm::vec3 wMouse = glm::unProject(sMouse, MV, P, glm::vec4(0, 0, width, height));
		glm::vec3 prevMouse = mouseJoint.GetMouseAnchor();

		if (glm::length2(prevMouse - wMouse) > 1.0f)
		{
			picked = false;
			return;
		}

		mouseJoint.SetMouseAnchor(wMouse);
	}
}

void Simulation::OnMouseScroll(GLFWwindow* window, double dx, double dy)
{
}

void Simulation::OnMouseButton(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS)
	{
		// read pixel depth at mouse click position - gives screen space 3D point
		float mouseZ = 0;
		glReadPixels(mouseX, height - mouseY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &mouseZ);

		// convert the point from screen space to world space
		glm::vec3 sMouse = glm::vec3(mouseX, height - mouseY, mouseZ);
		glm::mat4 MV = Camera::GetInstance().GetViewMatrix();
		glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
		glm::vec3 wMouse = glm::unProject(sMouse, MV, P, glm::vec4(0, 0, width, height));

		// check if mouse point with collider
		for (int i = 0; i < colliders.size(); i++)
		{
			Collider*c = colliders[i];
			
			if (c->GetBody()->GetInvMass() == 0.0f)
				continue;

			if (QueryPoint(c, wMouse))
			{
				mouseJoint = MouseJoint(c->GetBody(), wMouse);
				mouseJoint.SetMouseAnchor(wMouse);
				picked = true;
			}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		picked = false;
	}
}

void Simulation::OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (key == GLFW_KEY_RIGHT_ALT && action == GLFW_PRESS)
		debugDraw = !debugDraw;

	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
			keys[key] = true;
		else if (action == GLFW_RELEASE)
			keys[key] = false;
	}

	if (keys[GLFW_KEY_W])
		Camera::GetInstance().Move(Camera::GetInstance().GetCamZ());
	if (keys[GLFW_KEY_S])
		Camera::GetInstance().Move(-Camera::GetInstance().GetCamZ());
	if (keys[GLFW_KEY_A])
		Camera::GetInstance().Move(-Camera::GetInstance().GetCamX());
	if (keys[GLFW_KEY_D])
		Camera::GetInstance().Move(Camera::GetInstance().GetCamX());
	if (keys[GLFW_KEY_Q])
		Camera::GetInstance().Move(Camera::GetInstance().GetCamY());
	if (keys[GLFW_KEY_Z])
		Camera::GetInstance().Move(-Camera::GetInstance().GetCamY());

	if (keys[GLFW_KEY_P])
		pauseStep = !pauseStep;
	if (keys[GLFW_KEY_N])
		advanceStep = true;
}

void Simulation::Step(const float dt)
{
	for (int i = 0; i < bodies.size(); i++)
	{
		bodies[i].Update(dt);
	}

	// To Do: Exploit frame coherence to cache contacts 
	// instead of rebuilding them each frame
	for (int i = 0; i < manifolds.size(); i++)
		manifolds[i].contacts.clear();
	manifolds.clear();

	// To Do : Broadphase

	for (int iA = 0; iA < colliders.size(); iA++)
	{
		for (int iB = iA + 1; iB < colliders.size(); iB++)
		{
			DetectCollision(manifolds, colliders[iA], colliders[iB]);
		}
	}

	for (int i = 0; i < VELOCITY_ITERS; i++)
	{
		for (auto manifold : manifolds)
		{
			manifold.SolveVelocities();
		}
	}

	for (int i = 0; i < POSITION_ITERS; i++)
	{
		for (auto manifold : manifolds)
		{
			manifold.SolvePositions();
		}
	}

	for (auto joint : posJoints)
	{
		joint.Solve();
	}

	if (picked)
		mouseJoint.Solve();
}

void Simulation::Update()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Physics Update
	const static float dt = 1.0f / 60.0f;

	if (!pauseStep)
		Step(dt);
	else
	{
		if (advanceStep)
		{
			Step(dt);
			advanceStep = false;
		}
	}

	// Graphics update
	static glm::mat4 T(1), R(1), S(1), M(1), MVP(1);
	glm::mat4 V = Camera::GetInstance().GetViewMatrix();
	glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();

	/*for (auto collider : colliders)
	{
		collider->Render();
	}*/
	for (auto b : bodies)
	{
		b.Render();
	}

	for (auto m : manifolds)
	{
		for (auto contact : m.contacts)
		{
			T = glm::translate(contact.GetPosition());
			S = glm::scale(glm::vec3(0.1f));
			M = T * S;
			MVP = P * V * M;
			boxModel->SetMVP(MVP);
			boxModel->Render();
		}
	}

	if (panLeft)
	{
		yaw += MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panRight)
	{
		yaw -= MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panTop)
	{
		pitch += MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panBot)
	{
		pitch -= MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
}

Simulation::~Simulation()
{}