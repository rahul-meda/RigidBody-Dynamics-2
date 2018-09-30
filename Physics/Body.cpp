
#pragma once

#include "Collider.h"
#include "Body.h"

namespace Physics
{
	Body::Body()
		:invMass(1.0),
		localInvInertia(0),
		invInertia(0),
		density(1.0f),
		restitution(0.3f),
		friction(0.7f),
		position(0),
		orientation(0,1,0,0),
		R(1),
		velocity(0),
		angularVelocity(0),
		forceSum(0),
		torqueSum(0)
	{}

	void Body::SetMass(const float m)
	{
		if (m == 0)	// static body
			invMass = 0;
		else
			invMass = 1.0f / m;
	}

	float Body::GetMass() const
	{
		if (invMass == 0)
			return 0;

		return 1.0f / invMass;
	}

	void Body::SetInvMass(const float im)
	{
		invMass = im;
	}

	float Body::GetInvMass() const
	{
		return invMass;
	}

	void Body::SetInertia(const glm::mat3& inertia)
	{
		localInvInertia = glm::inverse(inertia);
	}

	glm::mat3 Body::GetInertia() const
	{
		return invInertia;
	}

	void Body::SetDensity(const float d)
	{
		density = d;
	}

	float Body::GetDensity() const
	{
		return density;
	}

	void Body::SetRestitution(const float e)
	{
		restitution = e;
	}

	float Body::GetRestitution() const
	{
		return restitution;
	}

	void Body::SetFriction(const float u)
	{
		friction = u;
	}

	float Body::GetFriction() const
	{
		return friction;
	}

	void Body::SetPosition(const glm::vec3& pos)
	{
		position = pos;
	}

	glm::vec3 Body::GetPosition() const
	{
		return position;
	}

	void Body::SetOrientation(const glm::quat& o)
	{
		orientation = glm::normalize(o);
	}

	glm::quat Body::GetOrientation() const
	{
		return orientation;
	}

	void Body::SetVelocity(const glm::vec3& vel)
	{
		velocity = vel;
	}

	glm::vec3 Body::GetVelocity() const
	{
		return velocity;
	}

	void Body::SetAngularVelocity(const glm::vec3& w)
	{
		angularVelocity = w;
	}

	glm::vec3 Body::GetAngularVelocity() const
	{
		return angularVelocity;
	}

	const glm::vec3 Body::LocalToGlobalVec(const glm::vec3& v) const
	{
		return R * v;
	}

	const glm::vec3 Body::GlobalToLocalVec(const glm::vec3& v) const
	{
		return glm::transpose(R) * v;
	}

	const glm::vec3 Body::LocalToGlobalPoint(const glm::vec3& p) const
	{
		return position + R * p;
	}

	const glm::vec3 Body::GlobalToLocalPoint(const glm::vec3& p) const
	{
		return glm::transpose(R) * (p - position);
	}

	void Body::AddCollider(Collider* collider)
	{
		collider->SetBody(this);

		if (invMass == 0) return;

		colliders.push_back(collider);

		localCentroid = glm::vec3(0.0f);
		invMass = 0.0f;

		collider->CalculateMass();

		float mass = 0.0f;		// mass of the body

		for (Collider* c : colliders)
		{
			mass += c->GetMass();

			localCentroid += c->GetMass() * c->GetCentroid();
		}

		assert(mass != 0);

		invMass = 1.0f / mass;

		localCentroid *= invMass;
		centroid = LocalToGlobalPoint(localCentroid);

		glm::mat3 inertiaLocal(0);
		for (Collider* c : colliders)
		{
			glm::vec3 r = localCentroid - c->GetCentroid();
			float rDotr = glm::dot(r, r);
			glm::mat3 rOutr = glm::outerProduct(r, r);

			// Parallel axis theorem
			// Accumulate local inertia tensors
			inertiaLocal += c->GetInertia() + c->GetMass() * (rDotr * glm::mat3(1.0) - rOutr);
		}

		localInvInertia = glm::inverse(inertiaLocal);
	}

	void Body::ApplyForce(const glm::vec3& force)
	{
		forceSum += force;
	}

	void Body::ApplyForce(const glm::vec3& force, const glm::vec3& p)
	{
		forceSum += force;
		torqueSum += glm::cross(p - centroid, force);
	}

	void Body::Update(const float dt)
	{
		if (invMass == 0.0)
			return;

		velocity += invMass * forceSum * dt;
		angularVelocity += invInertia * torqueSum * dt;

		forceSum = glm::vec3(0);
		torqueSum = glm::vec3(0);

		centroid += velocity * dt;
		orientation += 0.5f * glm::quat(0, angularVelocity) * orientation * dt;

		orientation = glm::normalize(orientation);
		R = glm::toMat3(orientation);

		invInertia = glm::transpose(R) * localInvInertia * R;

		position = centroid - LocalToGlobalVec(localCentroid);
	}
}