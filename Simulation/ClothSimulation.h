
#pragma once

#include "Simulation.h"
#include "Particle.h"

class Model;
class Particle;
class Spring;

class ClothSimulation : public Simulation
{
public:
	static ClothSimulation& GetInstance();

	void OnInit(GLFWwindow* window);

	void OnKeyInput(GLFWwindow* window, int key, int code, int action, int mods);

	void Step(const float dt);

	void Update();

private:
	ClothSimulation() {};

	std::vector<Particle*> nodes;
	std::vector<Spring*> springs;

	std::vector<glm::vec3> vertices;
	std::vector<int> indices;

	std::vector<ParticleContact> particleContacts;

	Model* particleModel;

	bool advStep;
};