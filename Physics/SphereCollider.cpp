
#include "Body.h"
#include "SphereCollider.h"

SphereCollider::SphereCollider(const float radius)
	: radius(radius)
{
	shape = Sphere;
}

float SphereCollider::GetRadius()
{
	return radius;
}

void SphereCollider::CalculateMass()
{
	centroid = glm::vec3(0.0f);

	float r2 = radius * radius;
	mass = body->GetDensity() * (4.0f / 3.0f) * (glm::pi<float>()) * r2 * radius;

	inertia = (2.0f / 5.0f) * mass * r2 * glm::mat3(1.0f);
}