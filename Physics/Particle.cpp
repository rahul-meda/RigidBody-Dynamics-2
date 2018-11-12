
#include "Particle.h"
#include "Spring.h"

#define GRAVITY 9.8f

Particle::Particle(const float mass, const glm::vec3& position)
	: position(position),
	prevPosition(position),
	velocity(0),
	forceSum(0)
{
	SetMass(mass);
}

void Particle::SetMass(const float mass)
{
	if (mass == 0)
		invMass = 0;
	else
		invMass = 1.0f / mass;
}

float Particle::GetMass() const
{
	if (invMass == 0)
		return 0;
	else
		return 1.0f / invMass;
}

float Particle::GetInvMass() const
{
	return invMass;
}

void Particle::SetPosition(const glm::vec3& pos)
{
	position = pos;
}

glm::vec3 Particle::GetPosition() const
{
	return position;
}

void Particle::SetVelocity(const glm::vec3& vel)
{
	velocity = vel;
}

glm::vec3 Particle::GetVelocity() const
{
	return velocity;
}

void Particle::AddForce(const glm::vec3& force)
{
	forceSum += force;
}

void Particle::Connect(Link* link)
{
	links.push_back(link);
}

glm::vec3 Particle::AccelerationFn(const glm::vec3& p, const glm::vec3& v)
{
	glm::vec3 r(0), dirCap(0), f(0);
	float l = 0.0f, l0 = 0.0f, k = 0.0f, C = 0.0f;
	for (Link* L : links)
	{
		r = L->B->position - p;
		l = glm::length(r);
		assert(l != 0);
		l0 = L->S->GetRestLength();
		k = L->S->GetStiffness();
		C = L->S->GetDampingRatio();
		dirCap = r / l;

		f += (k * (l - l0) + C * glm::dot(L->B->velocity - v, dirCap)) * dirCap;
	}

	return (f * invMass) + (GRAVITY * (glm::vec3(0, -1.0, 0)));
}

void Particle::Evaluate(const Derivative& in, Derivative& out, const float dt)
{
	glm::vec3 p = position + in.dx * dt;
	glm::vec3 v = velocity + in.dv * dt;

	out.dx = v;
	const static float k = 10.0f;
	const static float b = 0.1f;
	out.dv = AccelerationFn(p, v);
}

void Particle::Update(const float dt, const Integrator type)
{
	if (invMass == 0)
		return;

	switch (type)
	{
	case (Semi_Implicit_Euler) :
	{
		velocity += invMass * forceSum * dt;
		velocity += GRAVITY * (glm::vec3(0, -1.0, 0)) * dt;

		forceSum = glm::vec3(0);

		position += velocity * dt;
		break;
	}
	case (Position_Verlet) :
	{
		velocity += invMass * forceSum * dt;
		velocity += GRAVITY * (glm::vec3(0, -1.0, 0)) * dt;

		glm::vec3 diff = position - prevPosition;
		prevPosition = position;
		position += diff + (forceSum + GRAVITY * (glm::vec3(0, -1.0, 0))) * dt * dt;

		forceSum = glm::vec3(0);
		break;
	}
	case (RK4) :
	{
		Derivative a, b, c, d;

		Evaluate(Derivative(), a, 0.0f);
		Evaluate(a, b, 0.5f*dt);
		Evaluate(b, c, 0.5f*dt);
		Evaluate(c, d, dt);

		glm::vec3 dxdt = (1.0f / 6.0f) * (a.dx + 2.0f * (b.dx + c.dx) + d.dx);
		glm::vec3 dvdt = (1.0f / 6.0f) * (a.dv + 2.0f * (b.dv + c.dv) + d.dv);

		position += dxdt * dt;
		velocity += dvdt * dt;
		break;
	}
	default:
		assert(false);
		break;
	}
}