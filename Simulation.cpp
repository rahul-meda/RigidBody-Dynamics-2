
#include "Simulation.h"
#include "Poly.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Camera.h"

using namespace Graphics;

#define MOUSE_SENSITIVITY 0.1

namespace Demo
{
	Simulation::Simulation()
		:debugDraw(false), firstMouseCB(false)
	{
		panLeft = panRight = panBot = panTop = false;
	}

	void Simulation::OnInit(GLFWwindow* window)
	{
		glfwGetWindowSize(window, &width, &height);

		mouseX = width / 2.0f;
		mouseY = height / 2.0f;

		glClearColor(0.9, 0.9, 0.9, 0);

		// draw the pixel only if the object is closer to the viewer
		glEnable(GL_DEPTH_TEST); // enables depth-testing
		glDepthFunc(GL_LESS);    // interpret smaller values as closer

		Graphics::Camera::GetInstance().SetProjection(45.0f, (float)width / (float)height);

		Graphics::Camera::GetInstance().SetPosition(glm::vec3(0,1,0));

		std::vector<glm::vec3> vertices(8);
		std::vector<int> indices(36);

		vertices[0] = glm::vec3(-0.5f, -0.5f, -0.5f);
		vertices[1] = glm::vec3(0.5f, -0.5f, -0.5f);
		vertices[2] = glm::vec3(0.5f, 0.5f, -0.5f);
		vertices[3] = glm::vec3(-0.5f, 0.5f, -0.5f);
		vertices[4] = glm::vec3(-0.5f, -0.5f, 0.5f);
		vertices[5] = glm::vec3(0.5f, -0.5f, 0.5f);
		vertices[6] = glm::vec3(0.5f, 0.5f, 0.5f);
		vertices[7] = glm::vec3(-0.5f, 0.5f, 0.5f);

		int* id = &indices[0];
		//bottom face
		*id++ = 0; 	*id++ = 5; 	*id++ = 4;
		*id++ = 5; 	*id++ = 0; 	*id++ = 1;

		//top face
		*id++ = 3; 	*id++ = 7; 	*id++ = 6;
		*id++ = 3; 	*id++ = 6; 	*id++ = 2;

		//front face
		*id++ = 7; 	*id++ = 4; 	*id++ = 6;
		*id++ = 6; 	*id++ = 4; 	*id++ = 5;

		//back face
		*id++ = 2; 	*id++ = 1; 	*id++ = 3;
		*id++ = 3; 	*id++ = 1; 	*id++ = 0;

		//left face 
		*id++ = 3; 	*id++ = 0; 	*id++ = 7;
		*id++ = 7; 	*id++ = 0; 	*id++ = 4;

		//right face 
		*id++ = 6; 	*id++ = 5; 	*id++ = 2;
		*id++ = 2; 	*id++ = 5; 	*id++ = 1;

		cube = new Graphics::Poly(vertices, indices);
		static_cast<Poly*>(cube)->SetFrame();
		floor = new Graphics::Poly(vertices, indices);
		floor->SetColor(glm::vec3(0.2,0.2,0.7));
	}

	void Simulation::OnWindowResize(GLFWwindow* window, int width, int height)
	{
		this->width = width;
		this->height = height;
		glViewport(0, 0, width, height);

		Camera::GetInstance().SetProjection(45.0f, (float)width / height);
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
		float dy = y - mouseY;
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

		if (pitch > 89.0f) pitch = 89.0f;
		if (pitch < -89.0f) pitch = -89.0f;

		Camera::GetInstance().Rotate(yaw, pitch, 0);
	}

	void Simulation::OnMouseScroll(GLFWwindow* window, double dx, double dy)
	{
	}

	void Simulation::OnMouseButton(GLFWwindow* window, int button, int action, int mods)
	{

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
	}

	void Simulation::Update()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Physics Update

		// Graphics update
		glm::mat4 T(1), R(1), S(1), M(1);
		glm::mat4 V = Camera::GetInstance().GetViewMatrix();
		glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
		T = glm::translate(glm::vec3(0,0,0));
		R = glm::toMat4(glm::quat(0.0, glm::vec3(0,0,1)));
		S = glm::scale(glm::vec3(1.0));
		M = T * R * S;
		glm::mat4 MVP = P * V * M;
		cube->SetMVP(MVP);
		cube->Render();
		Poly* frame = static_cast<Poly*>(cube)->GetFrame();
		frame->SetMVP(MVP);
		frame->Render();

		frame = nullptr;

		T = glm::translate(glm::vec3(0,-1.0,0));
		S = glm::scale(glm::vec3(10.0,1.0,10.0));
		M = T * R * S;
		MVP = P * V * M;
		floor->SetMVP(MVP);
		floor->Render();

		// Debug Draw
		if (debugDraw)
		{
			// wire frame mode
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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
	{
		delete cube;
		delete floor;
	}
}