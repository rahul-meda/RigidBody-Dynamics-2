
#pragma once

#include "Simulation.h"

class RevoluteJointTest : public Simulation
{
public:
	static RevoluteJointTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	RevoluteJointTest() {};
};