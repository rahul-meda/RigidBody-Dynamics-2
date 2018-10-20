
#pragma once

#include "Simulation.h"

class FractureTest : public Simulation
{
public:
	static FractureTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

	void Update();

private:
	FractureTest() {};
};