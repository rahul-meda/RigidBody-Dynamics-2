
#pragma once

#include <glm/glm.hpp>
#include <vector>

class Model;
class Spring;
class Particle;

enum Integrator
{
	Semi_Implicit_Euler = 1,
	Position_Verlet,
	Velocity_Verlet,
	RK4
};

struct Derivative
{
	glm::vec3 dx;
	glm::vec3 dv;

	Derivative()
		: dx(0), dv(0)
	{}
};

struct Link
{
	Particle* B;
	Spring* S;

	Link(Particle* B, Spring* S)
		: B(B), S(S)
	{}
};

struct ParticleContact
{
	Particle* p;
	glm::vec3 normal;
	float penetration;

	ParticleContact(Particle* p, const glm::vec3& normal, const float penetration)
		: p(p), normal(normal), penetration(penetration)
	{}
};

class Particle
{
private:
	float invMass;
	glm::vec3 position;
	glm::vec3 prevPosition;
	glm::vec3 velocity;
	glm::vec3 forceSum;

	// list of particles connected to this particle
	std::vector<Link*> links;

public:
	Particle(const float mass, const glm::vec3& position);

	void SetMass(const float mass);
	float GetMass() const;
	float GetInvMass() const;

	void SetPosition(const glm::vec3& pos);
	glm::vec3 GetPosition() const;

	void SetVelocity(const glm::vec3& vel);
	glm::vec3 GetVelocity() const;

	void AddForce(const glm::vec3& force);

	///////// RK4 helpers ////////////////

	// add a spring b/w this and given particle
	void Connect(Link* link);

	// calclates the acceleration of this particle with given state
	glm::vec3 AccelerationFn(const glm::vec3& p, const glm::vec3& v);

	void Evaluate(const Derivative& in, Derivative& out, const float dt);

	///////// RK4 helpers ////////////////

	void Update(const float dt, const Integrator type = Semi_Implicit_Euler);
};