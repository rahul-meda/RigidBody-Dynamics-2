
#pragma once

#include "Simulation.h"

class PositionJointTest : public Simulation
{
public:
	static PositionJointTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	PositionJointTest() {};
};