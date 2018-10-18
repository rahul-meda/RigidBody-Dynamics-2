
#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "Model.h"

class Collider;

struct ModelData
{
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;
	std::vector<int> frameIndices;

	ModelData(){};

	ModelData(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices, const std::vector<int>& frameIndices)
		: vertices(vertices), indices(indices), frameIndices(frameIndices)
	{}
};

struct Velocity
{
	glm::vec3 v;
	glm::vec3 w;

	Velocity(const glm::vec3& v, const glm::vec3& w)
		: v(v), w(w)
	{}
};

struct Position
{
	glm::vec3 c;
	glm::quat q;

	Position(const glm::vec3& c, const glm::quat& q)
		: c(c), q(q)
	{}
};

class Body
{
private:
	float invMass;
	glm::mat3 localInvInertia;
	glm::mat3 invInertia;

	float density;
	float restitution;
	float friction;

	glm::vec3 localCentroid;
	glm::vec3 centroid;
	glm::vec3 position;
	glm::quat orientation;
	glm::mat3 R;

	glm::vec3 velocity;
	glm::vec3 angularVelocity;

	glm::vec3 forceSum;
	glm::vec3 torqueSum;

	std::vector<Collider*> colliders;

	Model* model;
	Model* frame;
	glm::vec3 color;
	int id;

public:
	Body();

	void SetMass(const float m);
	float GetMass() const;

	void SetInvMass(const float m);
	float GetInvMass() const;

	void SetInertia(const glm::mat3& inertia);
	glm::mat3 GetInvInertia() const;

	void SetDensity(const float d);
	float GetDensity() const;

	void SetRestitution(const float e);
	float GetRestitution() const;

	void SetFriction(const float u);
	float GetFriction() const;

	void SetCentroid(const glm::vec3& c);
	glm::vec3 GetCentroid() const;

	void SetPosition(const glm::vec3& pos);
	glm::vec3 GetPosition() const;

	void SetOrientation(const glm::quat& o);
	glm::quat GetOrientation() const;

	void SetVelocity(const glm::vec3& vel);
	glm::vec3 GetVelocity() const;

	void SetAngularVelocity(const glm::vec3& w);
	glm::vec3 GetAngularVelocity() const;

	void SetModelData(const ModelData& m);

	void SetColor(const glm::vec3& color);

	void SetID(const int i);
	int GetID() const;

	const glm::vec3 LocalToGlobalVec(const glm::vec3& v) const;

	const glm::vec3 GlobalToLocalVec(const glm::vec3& v) const;

	const glm::vec3 LocalToGlobalPoint(const glm::vec3& p) const;

	const glm::vec3 GlobalToLocalPoint(const glm::vec3& p) const;

	// transform from A's local space to this body's space
	const glm::vec3 LocalToLocalVec(Body* A, const glm::vec3& v) const;
	const glm::vec3 LocaltoLocalPoint(Body* A, const glm::vec3& p) const;

	void AddCollider(Collider* collider);

	// apply force at c.o.m
	void ApplyForce(const glm::vec3& force);

	// apply force at a point on the body (point given in body space)
	void ApplyForce(const glm::vec3& force, const glm::vec3& pt);

	void IntegrateVelocity(const float dt);

	void IntegratePosition(const float dt);

	void Update(const float dt);

	void Render();
};