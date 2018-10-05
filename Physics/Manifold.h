
#pragma once

#include <glm/glm.hpp>
#include "Geometry.h"

class Body;

// contact data for a collider pair
struct Manifold
{
	Body* A;			// the body pair
	Body* B;			// involved in this contact

	glm::vec3 position;		// the contact point in world space

	glm::vec3 normal;		// the directon in which the colliders must be spearated (by convention always from A to B)

	glm::vec3 tangent1;		// mutually perendicular directions for friction
	glm::vec3 tangent2;		// forming an othonormal basis with the contact normal

	float penetration;		// the amount of overlap b/w the colliders

	// for clamping the lagrangian
	float impulseSumN;
	float impulseSumT1;
	float impulseSumT2;

	Manifold(Body* A, Body* B, const glm::vec3& position, const glm::vec3& normal, const float penetration)
		: A(A), B(B), position(position), normal(normal), penetration(penetration)
	{
		ComputeBasis(normal, tangent1, tangent2);
	}
};