
#pragma once

#include "Simulation.h"

class Dominoes : public Simulation
{
public:
	static Dominoes& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	Dominoes() {};
};