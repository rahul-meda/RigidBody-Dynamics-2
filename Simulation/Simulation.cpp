
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
#include "Line.h"

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

	ModelData model;
	ParseObj("resources/cylinder.obj", mesh);
	mesh.GetModelData(model);
	cylinder = new Model(model.vertices, model.indices);
	CreateSphere(1.0, model);
	sphere = new Model(model.vertices, model.indices);
	CreateHemiSphere(1.0, model);
	hemiSphere = new Model(model.vertices, model.indices);

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
			if (colliders[iA]->GetBody()->GetTag() == colliders[iB]->GetBody()->GetTag() && colliders[iA]->GetBody()->GetTag() != "")
				continue;

			if (colliders[iA]->GetBody()->GetInvMass() == 0.0f && colliders[iB]->GetBody()->GetInvMass() == 0.0f)
				continue;

			if (colliders[iA]->GetShape() == Collider::Sphere || colliders[iB]->GetShape() == Collider::Sphere)
			{
				if (colliders[iA]->GetBody()->GetTag() == "teapot" || colliders[iB]->GetBody()->GetTag() == "teapot")
					continue;
			}

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

	for (int i = 0; i < 20; i++)
	{
		for (auto j : posJoints)
			j.Solve();
	}

	for (auto c : planeConstraints)
		c.Solve();

	for (auto j : revJoints)
		j.Solve();

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
	static glm::mat4 T(1), R(1), S(1), M(1), VP(1), MVP(1);
	ModelData model;
	glm::mat4 V = Camera::GetInstance().GetViewMatrix();
	glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
	VP = P * V;

	for (auto b : bodies)
	{
		b.Render();
	}

	for (auto m : manifolds)
	{
		for (auto contact : m.contacts)
		{
			T = glm::translate(contact.GetPosition());
			S = glm::scale(glm::vec3(0.03f));
			M = T * S;
			MVP = VP * M;
			sphere->SetMVP(MVP);
			sphere->Render();

			// tangents
			/*std::vector<glm::vec3> verts = { contact.GetPosition(), contact.GetPosition() + contact.GetTangent(0) };
			std::vector<int> ids = { 0, 1 };
			Line* line = new Line(verts, ids);
			line->SetMVP(VP);
			line->SetColor(glm::vec3(0, 1, 0));
			line->Render();
			verts = { contact.GetPosition(), contact.GetPosition() + contact.GetTangent(1) };
			line = new Line(verts, ids);
			line->SetMVP(VP);
			line->SetColor(glm::vec3(0, 1, 0));
			line->Render();
			delete line;*/
		}
	}

	for (auto j : revJoints)
	{
		T = glm::translate(j.GetAnchor());
		glm::vec3 axis = j.GetAxis();
		if (axis.x > 0.9f)
			R = glm::toMat4(glm::angleAxis(1.57f, glm::vec3(0,0,1.0)));
		else if (axis.z > 0.9f)
			R = glm::toMat4(glm::angleAxis(1.57f, glm::vec3(1.0, 0, 0)));
		else
			R = glm::mat4(1.0);
		S = glm::scale(glm::vec3(0.05, 0.3, 0.05));
		cylinder->SetMVP(VP * T * R * S);
		cylinder->SetColor(glm::vec3(0.7, 0.7, 0.6));
		cylinder->Render();
	}

	for (auto j : posJoints)
	{
		glm::vec3 p1 = j.GetAnchorA();
		glm::vec3 p2 = j.GetBodyA()->GetCentroid();
		M = glm::translate(p1) * glm::scale(glm::vec3(0.0125f));
		MVP = VP * M;
		sphere->SetColor(glm::vec3(1, 0, 0));
		sphere->SetMVP(MVP);
		sphere->Render();
		/*CreateLine(p1, p2, model);
		Model* line = new Model(model.vertices, model.indices);
		line->SetPrimitive(GL_LINES);
		line->SetMVP(VP);
		line->SetColor(glm::vec3(1, 0, 0));
		line->Render();*/

		p1 = j.GetAnchorB();
		p2 = j.GetBodyB()->GetCentroid();
		glm::quat q = j.GetBodyB()->GetOrientation();
		//R = glm::toMat4(q) * glm::rotate(-1.5714f, glm::vec3(0, 0, 1));
		M = glm::translate(p1) * R * glm::scale(glm::vec3(0.025f));
		MVP = VP * M;
		hemiSphere->SetColor(glm::vec3(0.643, 0.827, 0.435));
		hemiSphere->SetMVP(MVP);
		hemiSphere->Render();
		/*CreateLine(p1, p2, model);
		line = new Model(model.vertices, model.indices);
		line->SetPrimitive(GL_LINES);
		line->SetMVP(VP);
		line->SetColor(glm::vec3(0.643, 0.827, 0.435));
		line->Render();
		delete line;*/
	}

	float s = 3.0f;
	if (panLeft)
	{
		yaw += s * MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panRight)
	{
		yaw -= s * MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panTop)
	{
		pitch -= s * MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
	if (panBot)
	{
		pitch += s * MOUSE_SENSITIVITY;
		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}
}

Simulation::~Simulation()
{}