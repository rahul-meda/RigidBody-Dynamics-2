
#pragma once

#include "Body.h"

namespace Physics
{
	Body::Body()
		:invMass(0),
		localInvInertia(0),
		invInertia(0),
		position(0),
		orientation(0,1,0,0),
		R(1),
		velocity(0),
		angularVelocity(0),
		forceSum(0),
		torqueSum(0),
		model(nullptr)
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

	void Body::SetPosition(const glm::vec3 pos)
	{
		position = pos;
	}

	glm::vec3 Body::GetPosition() const
	{
		return position;
	}

	void Body::SetOrientation(const glm::quat o)
	{
		orientation = glm::normalize(o);
	}

	glm::quat Body::GetOrientation() const
	{
		return orientation;
	}

	void Body::SetVelocity(const glm::vec3 vel)
	{
		velocity = vel;
	}

	glm::vec3 Body::GetVelocity() const
	{
		return velocity;
	}

	void Body::SetAngularVelocity(const glm::vec3 w)
	{
		angularVelocity = w;
	}

	glm::vec3 Body::GetAngularVelocity() const
	{
		return angularVelocity;
	}

	void Body::SetModel(Graphics::Model* m)
	{
		model = m;
	}

	Graphics::Model* Body::GetModel() const
	{
		return model;
	}

	void Body::Update(const float dt)
	{
		if (invMass == 0.0)
			return;

		velocity += invMass * forceSum * dt;
		angularVelocity += invInertia * torqueSum * dt;

		forceSum = glm::vec3(0);
		torqueSum = glm::vec3(0);

		position += velocity * dt;
		orientation += 0.5f * glm::quat(0, angularVelocity) * orientation * dt;

		orientation = glm::normalize(orientation);
		R = glm::toMat3(orientation);
	}
}