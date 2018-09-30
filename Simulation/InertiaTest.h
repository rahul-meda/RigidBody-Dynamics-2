
#pragma once

#include "Simulation.h"

namespace Simulation
{
	class InertiaTest : public Simulation
	{
	public:
		static InertiaTest& GetInstance();

		void OnInit(GLFWwindow* window);

		void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

	private:
		InertiaTest() {};
	};
}