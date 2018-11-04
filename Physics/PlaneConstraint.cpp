
#include "PlaneConstraint.h"
#include "ConstraintCommon.h"

PlaneConstraint::PlaneConstraint(Body* A, const glm::vec3& anchor, const glm::vec3& axis)
	: A(A), localAnchor(anchor), axis(axis)
{}

void PlaneConstraint::CalculateJacobian(Jacobian& J, const glm::vec3& r)
{
	J = Jacobian(-axis, -glm::cross(r, axis), glm::vec3(0.0f), glm::vec3(0.0f));
}

float PlaneConstraint::CalculateBias(float correction)
{
	const static float B = 0.179f;
	const static float inv_dt = 60.0f;

	return (B * inv_dt * correction);
}

void PlaneConstraint::Solve()
{
	glm::vec3 anchor = A->LocalToGlobalPoint(localAnchor);
	glm::vec3 r = anchor - A->GetCentroid();

	Jacobian J;
	CalculateJacobian(J, r);

	float effMass = glm::dot(J.L1, J.L1) * A->GetInvMass() + glm::dot(J.A1, J.A1 * A->GetInvInertia());

	float JV = glm::dot(J.L1, A->GetVelocity()) + glm::dot(J.A1, A->GetAngularVelocity());
	float lambda = (-JV / effMass);

	glm::vec3 v = A->GetVelocity();
	glm::vec3 w = A->GetAngularVelocity();
	v += lambda * J.L1 * A->GetInvMass();
	w += lambda * J.A1 * A->GetInvInertia();
	A->SetVelocity(v);
	A->SetAngularVelocity(w);
}