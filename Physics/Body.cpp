
#pragma once

#include "Collider.h"
#include "Body.h"
#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "HullCollider.h"

#define GRAVITY 9.8f

Body::Body()
	:invMass(1.0),
	localInvInertia(0),
	invInertia(0),
	density(1.0f),
	restitution(0.3f),
	friction(0.2f),
	position(0),
	orientation(1, 0, 0, 0),
	R(1),
	velocity(0),
	angularVelocity(0),
	forceSum(0),
	torqueSum(0),
	color(0.4, 0.9, 0.1)
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

glm::mat3 Body::GetInvInertia() const
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

void Body::SetCentroid(const glm::vec3& c)
{
	centroid = c;
}

glm::vec3 Body::GetCentroid() const
{
	return centroid;
}

void Body::SetPosition(const glm::vec3& pos)
{
	position = pos;

	centroid = position + LocalToGlobalVec(localCentroid);
}

glm::vec3 Body::GetPosition() const
{
	return position;
}

void Body::SetOrientation(const glm::quat& o)
{
	orientation = glm::normalize(o);
	R = glm::mat3(orientation);
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

void Body::SetModelData()
{
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;
	std::vector<int> frameIndices;
	int offset = 0;

	for (Collider* c : colliders)
	{
		for (glm::vec3 v : c->GetModelData().vertices)
		{
			vertices.push_back(v + c->GetPosition());
		}
	}
	for (Collider* c : colliders)
	{
		for (int i : c->GetModelData().indices)
		{
			indices.push_back(i + offset);
		}
		offset = c->GetModelData().vertices.size();
	}
	offset = 0;
	for (Collider* c : colliders)
	{
		for (int i : c->GetModelData().frameIndices)
		{
			frameIndices.push_back(i + offset);
		}
		offset = c->GetModelData().vertices.size();
	}

	model = new Model(vertices, indices);
	frame = new Model(vertices, frameIndices);
	frame->SetPrimitive(GL_LINES);
}

void Body::SetColor(const glm::vec3& color)
{
	this->color = color;
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

const glm::vec3 Body::LocalToLocalVec(Body* A, const glm::vec3& v) const
{
	return GlobalToLocalVec(A->LocalToGlobalVec(v));
}

const glm::vec3 Body::LocaltoLocalPoint(Body* A, const glm::vec3& p) const
{
	return GlobalToLocalPoint(A->LocalToGlobalPoint(p));
}

void Body::AddCollider(Collider* collider)
{
	collider->SetBody(this);
	colliders.push_back(collider);

	if (invMass == 0) return;

	localCentroid = glm::vec3(0.0f);
	invMass = 0.0f;

	switch (collider->GetShape())
	{
	case (Collider::Hull) :
		static_cast<HullCollider*>(collider)->CalculateMass();
		break;
	}

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
	torqueSum += glm::cross(LocalToGlobalPoint(p) - centroid, force);
}

void Body::IntegrateVelocity(const float dt)
{
	invInertia = glm::transpose(R) * localInvInertia * R;

	velocity += invMass * forceSum * dt;
	velocity += GRAVITY * (glm::vec3(0, -1, 0)) * dt;
	angularVelocity += invInertia * torqueSum * dt;

	// damp velocity?

	forceSum = glm::vec3(0);
	torqueSum = glm::vec3(0);
}

void Body::IntegratePosition(const float dt)
{
	centroid += velocity * dt;
	orientation += 0.5f * glm::quat(0, angularVelocity) * orientation * dt;

	orientation = glm::normalize(orientation);
	R = glm::toMat3(orientation);
	position = centroid - LocalToGlobalVec(localCentroid);
}

void Body::Update(const float dt)
{
	if (invMass == 0.0)
		return;

	invInertia = glm::transpose(R) * localInvInertia * R;

	velocity += invMass * forceSum * dt;
	velocity += GRAVITY * (glm::vec3(0, -1, 0)) * dt;
	angularVelocity += invInertia * torqueSum * dt;

	forceSum = glm::vec3(0);
	torqueSum = glm::vec3(0);

	centroid += velocity * dt;
	orientation += 0.5f * glm::quat(0.0f, angularVelocity) * orientation * dt;

	orientation = glm::normalize(orientation);
	R = glm::toMat3(orientation);

	position = centroid - LocalToGlobalVec(localCentroid);
}

void Body::Render()
{
	static glm::mat4 T(1), R(1), S(1), M(1), V(1), P(1), VP(1), MVP(1);
	V = Camera::GetInstance().GetViewMatrix();
	P = Camera::GetInstance().GetProjectionMatrix();

	T = glm::translate(position);
	R = glm::toMat4(orientation);
	M = T * R * S;
	VP = P * V;
	MVP = VP * M;
	model->SetMVP(MVP);
	model->SetColor(color);
	model->Render();

	frame->SetMVP(MVP);
	frame->SetColor(glm::vec3(0.9));
	frame->Render();
}