
#include "Simulation.h"
#include "Poly.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Camera.h"
#include <memory>

using namespace Graphics;

#define MOUSE_SENSITIVITY 0.1

namespace Demo
{
	Simulation::Simulation()
		:debugDraw(false), firstMouseCB(false), pauseStep(true), advanceStep(false)
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

		boxModel = new Graphics::Poly(vertices, indices);
		static_cast<Graphics::Poly*>(boxModel)->SetFrame();
		
		box.SetModel(boxModel);
		box.SetPosition(glm::vec3(0,2.0,0));
		box.SetVelocity(glm::vec3(1.0,0,0));
		box.SetMass(1.0f);

		floor.SetModel(boxModel);
		floor.SetPosition(glm::vec3(0));
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

		if (pitch > 89.0f) pitch = 89.0f;
		if (pitch < -89.0f) pitch = -89.0f;
		if (yaw > 89.0f) yaw = 89.0f;
		if (yaw < -89.0f) yaw = -89.0f;

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

		if (keys[GLFW_KEY_P])
			pauseStep = !pauseStep;
		if (keys[GLFW_KEY_N])
			advanceStep = true;
	}

	void Simulation::Step(const float dt)
	{
		box.Update(dt);
		floor.Update(dt);
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
		glm::mat4 T(1), R(1), S(1), M(1);
		glm::mat4 V = Camera::GetInstance().GetViewMatrix();
		glm::mat4 P = Camera::GetInstance().GetProjectionMatrix();
		T = glm::translate(box.GetPosition());
		R = glm::toMat4(box.GetOrientation());
		S = glm::scale(glm::vec3(1.0));
		M = T * R * S;
		glm::mat4 MVP = P * V * M;
		box.GetModel()->SetMVP(MVP);
		box.GetModel()->SetColor(glm::vec3(0.3, 0.9, 0.3));
		box.GetModel()->Render();
		Poly* frame = static_cast<Poly*>(box.GetModel())->GetFrame();
		frame->SetMVP(MVP);
		frame->Render();

		T = glm::translate(floor.GetPosition());
		R = glm::toMat4(floor.GetOrientation());
		S = glm::scale(glm::vec3(10.0,1.0,10.0));
		M = T * R * S;
		MVP = P * V * M;
		floor.GetModel()->SetMVP(MVP);
		floor.GetModel()->SetColor(glm::vec3(0.7,0.7,0.9));
		floor.GetModel()->Render();
		frame->SetMVP(MVP);
		frame->Render();

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
		delete boxModel;
	}
}