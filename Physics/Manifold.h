
#pragma once

#include <glm/glm.hpp>

namespace Physics
{
	class Collider;

	// contact data for a collider pair
	struct Manifold
	{
		Collider* A;			// the collider pair
		Collider* B;			// involved in this contact

		glm::vec3 contact;		// the contact point in world space

		glm::vec3 normal;		// the directon in which the colliders must be spearated (by convention always from A to B)

		glm::vec3 tangent1;		// mutually perendicular directions for friction
		glm::vec3 tangent2;		// forming an othonormal basis with the contact normal

		float penetration;		// the amount of overlap b/w the colliders

		// for clamping the lagrangian
		float impulseSumN;
		float impulseSumT1;
		float impulseSumT2;
	};

	// Compute a consistent orthonormal basis with the given vector
	void ComputeBasis(const glm::vec3& i, glm::vec3& j, glm::vec3& k)
	{
		// Ref: http://box2d.org/2014/02/computing-a-basis/
		// At least one component of a unit vector should be >= 0.57735f
		if (std::abs(i.x) >= 0.57735f)
			j = glm::vec3(i.y, -j.x, 0);
		else
			j = glm::vec3(0, i.z, -i.y);

		j = glm::normalize(j);
		k = glm::cross(i, j);
		k = glm::normalize(k);
	}
}