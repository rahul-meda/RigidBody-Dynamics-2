
#pragma once

#include "Simulation.h"

class FrictionTest : public Simulation
{
public:
	static FrictionTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	FrictionTest() {};
};