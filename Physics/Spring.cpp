
#include "Spring.h"
#include "Particle.h"

#define C 0.01f	// damping ratio

Spring::Spring(Particle* A, Particle* B, const float k)
	: A(A), B(B), k(k)
{
	l0 = glm::length(B->GetPosition() - A->GetPosition());
	assert(l0 != 0);

	A->Connect(new Link(B, this));
	B->Connect(new Link(A, this));
}

Particle* Spring::GetParticleA() const
{
	return A;
}

Particle* Spring::GetParticleB() const
{
	return B;
}

float Spring::GetRestLength() const
{
	return l0;
}

float Spring::GetStiffness() const
{
	return k;
}

float Spring::GetDampingRatio() const
{
	return C;
}

void Spring::Solve()
{
	glm::vec3 r = B->GetPosition() - A->GetPosition();
	float l = glm::length(r);	// current length
	assert(l != 0);
	glm::vec3 dirCap = r / l;
		
	glm::vec3 F = (k * (l - l0) + C * glm::dot(B->GetVelocity() - A->GetVelocity(), dirCap)) * dirCap;

	A->AddForce(F);
	B->AddForce(-F);
}