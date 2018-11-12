
#pragma once

class Particle;

class Spring
{
private:
	float l0;	// rest length

	float k;	// stiffness

	Particle* A;
	Particle* B;

public:
	Spring(Particle* A, Particle* B, const float k);

	Particle* GetParticleA() const;
	Particle* GetParticleB() const;

	float GetRestLength() const;

	float GetStiffness() const;

	float GetDampingRatio() const;

	void Solve();
};