
#pragma once

#include "Simulation.h"

class CompositeBodyTest : public Simulation
{
public:
	static CompositeBodyTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	CompositeBodyTest() {};
};