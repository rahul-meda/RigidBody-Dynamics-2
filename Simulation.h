
#pragma once

#include <vector>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Model.h"

namespace Demo
{
	class Simulation
	{
	protected:
		// window dimensions
		int width;
		int height;

		// camera transformation
		float yaw;
		float pitch;

		// mouse co-ordinates
		float mouseX;
		float mouseY;

		// for panning the scene on corner scroll
		bool panLeft, panRight, panTop, panBot;

		// to check for first mouse callback
		bool firstMouseCB;

		// store the state of pressed keys
		bool keys[1024];

		// toggle drawing wire-frame mode
		bool debugDraw;

		Graphics::Model* cube;
		Graphics::Model* floor;

	public:
		Simulation();

		virtual void OnInit(GLFWwindow* window);

		void OnWindowResize(GLFWwindow* window, int width, int height);

		void OnMouseMove(GLFWwindow* window, double x, double y);

		void OnMouseScroll(GLFWwindow* window, double dx, double dy);

		void OnMouseButton(GLFWwindow* window, int button, int action, int mods);

		virtual void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

		virtual void Update();
	};
}