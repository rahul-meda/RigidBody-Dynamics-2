
#pragma once

#include "Simulation.h"

class LemonadeMachine : public Simulation
{
public:
	static LemonadeMachine& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

	void Update();

private:
	LemonadeMachine() {};
};