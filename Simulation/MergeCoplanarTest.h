
#pragma once

#include "Simulation.h"

class MergeCoplanarTest : public Simulation
{
public:
	static MergeCoplanarTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	MergeCoplanarTest() {};
};