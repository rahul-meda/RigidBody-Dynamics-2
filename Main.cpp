
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include "Simulation/Simulations.h"

Simulation* sim = &LemonadeMachine::GetInstance();

void OnWindowResize(GLFWwindow* window, int width, int height)
{
	sim->OnWindowResize(window, width, height);
}

void OnMouseMove(GLFWwindow* window, double x, double y)
{
	sim->OnMouseMove(window, x, y);
}

void OnMouseScroll(GLFWwindow* window, double dx, double dy)
{
	sim->OnMouseScroll(window, dx, dy);
}

void OnMouseButton(GLFWwindow* window, int button, int action, int mods)
{
	sim->OnMouseButton(window, button, action, mods);
}

void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods)
{
	sim->OnKeyInput(window, key, code, action, mods);
}

int main()
{
	// Init GLFW
	if (!glfwInit())
	{
		std::cout << "Failed to initialize GLFW" << std::endl;
	}

	// Init window
	GLFWwindow* window = glfwCreateWindow(800, 600, "Combo", nullptr, nullptr);
	//GLFWwindow* window = glfwCreateWindow(1920, 1080, "Combo", nullptr, nullptr);
	//GLFWwindow* window = glfwCreateWindow(1920, 1080, "Combo", glfwGetPrimaryMonitor(), nullptr);
	if (window == nullptr)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();

		return 1;
	}
	glfwMakeContextCurrent(window);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	sim->OnInit(window);

	glfwSetWindowSizeCallback(window, OnWindowResize);
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetCursorPosCallback(window, OnMouseMove);
	//glfwSetScrollCallback(window, OnMouseScroll);
	glfwSetMouseButtonCallback(window, OnMouseButton);
	glfwSetKeyCallback(window, OnKeyInput);

	// Game Loop
	while (!glfwWindowShouldClose(window))
	{
		// register callback events
		glfwPollEvents();

		// Update Systems
		sim->Update();

		glfwSwapBuffers(window);
	}

	std::cout << "Window closed" << std::endl;
	glfwTerminate();

	return 0;
}