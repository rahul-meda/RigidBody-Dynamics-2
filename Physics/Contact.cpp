
#include "Contact.h"

#define BAUMGARTE 0.2f
#define VELOCITYSLOP 0.5f
#define POSITIONSLOP 0.005f

Contact::Contact(Body* A, Body* B, const glm::vec3& position, const glm::vec3& normal, const float penetration)
	: A(A), B(B), position(position), normal(normal), penetration(penetration),
	impulseSumN(0.0f), impulseSumT1(0.0f), impulseSumT2(0.0f)
{
	ComputeBasis(normal, tangent1, tangent2);

	rA = position - A->GetCentroid();
	rB = position - B->GetCentroid();

	CalculateJacobian();
	CalculateBias();
	effMass = CalculateEffectiveMass(J, A, B);
}

float Contact::CalculateSeparatingVelocity() const
{
	glm::vec3 vA = A->GetVelocity();
	vA += glm::cross(A->GetAngularVelocity(), rA);

	glm::vec3 vB = B->GetVelocity();
	vB += glm::cross(B->GetAngularVelocity(), rB);

	return glm::dot((vB - vA), normal);
}

void Contact::CalculateJacobian()
{
	J = Jacobian(-normal, -glm::cross(rA, normal), normal, glm::cross(rB, normal));
}

void Contact::CalculateBias()
{
	bias = 0.0f;
	// restitution
	float e = (A->GetRestitution() + B->GetRestitution()) * 0.5f;
	float vSep = CalculateSeparatingVelocity();
	bias += e * (glm::max(vSep - VELOCITYSLOP, 0.0f));
}

void Contact::SolveVelocities(Velocity& vA, Velocity& vB, const float dt = (1.0f/60.0f))
{
	float lambda = CalculateLagrangian(J, vA, vB, effMass, bias);

	float oldImpulse = impulseSumN;
	impulseSumN = glm::max(0.0f, impulseSumN + lambda);
	lambda = impulseSumN - oldImpulse;

	vA.v += lambda * J.L1 * A->GetInvMass();
	vA.w += lambda * J.A1 * A->GetInvInertia();
	vB.v += lambda * J.L2 * B->GetInvMass();
	vB.w += lambda * J.A2 * B->GetInvInertia();
}

void Contact::SolvePositions(Position& pA, Position& pB)
{
	float K = CalculateEffectiveMass(J, A, B);
	float C = -BAUMGARTE * (glm::max(penetration - POSITIONSLOP, 0.0f));
	float lambda = K > 0.0f ? -C / K : 0.0f;
	glm::vec3 P = lambda * normal;

	pA.c -= P * A->GetInvMass();
	pA.q += 0.5f * glm::quat(0.0f, A->GetInvInertia() * glm::cross(rA, -P)) * pA.q;
	pA.q = glm::normalize(pA.q);
	pB.c += P * B->GetInvMass();
	pB.q += 0.5f * glm::quat(0.0f, B->GetInvInertia() * glm::cross(rB, P)) * pA.q;
	pB.q = glm::normalize(pB.q);
}

void Manifold::SolveVelocities()
{
	assert(contacts.size() > 0);
	Body* A = contacts[0].A;
	Body* B = contacts[0].B;
	glm::vec3 v1 = A->GetVelocity();
	glm::vec3 w1 = A->GetAngularVelocity();
	glm::vec3 v2 = B->GetVelocity();
	glm::vec3 w2 = B->GetAngularVelocity();
	Velocity vA(v1, w1);
	Velocity vB(v2, w2);

	for (int i = 0; i < contacts.size(); i++)
	{
		contacts[i].SolveVelocities(vA, vB);
	}

	A->SetVelocity(vA.v);
	A->SetAngularVelocity(vA.w);
	B->SetVelocity(vB.v);
	B->SetAngularVelocity(vB.w);
}

void Manifold::SolvePositions()
{
	assert(contacts.size() > 0);
	Body* A = contacts[0].A;
	Body* B = contacts[0].B;
	glm::vec3 c1 = A->GetCentroid();
	glm::quat q1 = A->GetOrientation();
	glm::vec3 c2 = B->GetCentroid();
	glm::quat q2 = B->GetOrientation();
	Position pA(c1, q1);
	Position pB(c2, q2);

	for (int i = 0; i < contacts.size(); i++)
	{
		contacts[i].SolvePositions(pA, pB);
	}

	A->SetCentroid(pA.c);
	A->SetOrientation(pA.q);
	B->SetCentroid(pB.c);
	B->SetOrientation(pB.q);
}