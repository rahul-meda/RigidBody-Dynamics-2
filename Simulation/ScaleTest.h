
#pragma once

#include "Simulation.h"

class ScaleTest : public Simulation
{
public:
	static ScaleTest& GetInstance();

	void OnInit(GLFWwindow* window);

private:
	ScaleTest() {};
};