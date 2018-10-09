
#pragma once

#include <glm/glm.hpp>
#include "Geometry.h"
#include "ConstraintCommon.h"
#include "Body.h"

// Ref: Sequential Impules - Erin Catto – Iterative Dynamics with Temporal Coherence
// Contact constraint stops the bodies from interpenetrating at the contact point.
// Position is corrected by adding bias to the velocity constraint.
// Note: Zero restitution cannot be simulated with this correction method
// To Do: Post-stabilization - ref : box2D, https://www.cs.rutgers.edu/~dpai/papers/ClinePai03.pdf
// List of stabilization methods: https://github.com/erincatto/Box2D/blob/master/Box2D/Box2D/Dynamics/b2Island.cpp

// contact data for a collider pair
class Contact
{
private:
	Body* A;				// the body pair
	Body* B;				// involved in this contact

	glm::vec3 position;		// the contact point in world space
	glm::vec3 normal;		// the directon in which the colliders must be spearated (by convention always from A to B)
	glm::vec3 tangent1;		// mutually perendicular directions for friction
	glm::vec3 tangent2;		// forming an othonormal basis with the contact normal
	float penetration;		// the amount of overlap b/w the colliders

	// for clamping the lagrangian
	float impulseSumN;
	float impulseSumT1;
	float impulseSumT2;

	// vectors from each body's centroid to the contact point
	glm::vec3 rA;
	glm::vec3 rB;

	Jacobian J;

	// Bias is the "work" term in the velocity constraint equation
	// Accounts for position drift, and bounce(restitution)
	float bias;				

	float effMass;

private:
	// Projection of relative velocity along the normal 
	float CalculateSeparatingVelocity() const;

	// Calculates the Jacobian for this contact
	void CalculateJacobian();

	void CalculateBias();

public:
	Contact(Body* A, Body* B, const glm::vec3& position, const glm::vec3& normal, const float penetration);

	glm::vec3 GetPosition() const{ return position; }

	// Solves the contact by applying impulses
	void SolveVelocities(Velocity& A, Velocity& B, const float dt);

	void SolvePositions(Position& A, Position& B);

	friend struct Manifold;
};

struct Manifold
{
	std::vector<Contact> contacts;	// To Do : Duplicated data. Store pointer to contact and contact size

	void SolveVelocities();

	void SolvePositions();
};