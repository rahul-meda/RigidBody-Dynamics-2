
#pragma once

#include "Simulation.h"

class PlaneConstraintTest : public Simulation
{
public:
	static PlaneConstraintTest& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

private:
	PlaneConstraintTest() {};
};