
#pragma once

#include "Simulation.h"

class ParserTest : public Simulation
{
public:
	static ParserTest& GetInstance();

	void OnInit(GLFWwindow* window);

private:
	ParserTest() {};
};