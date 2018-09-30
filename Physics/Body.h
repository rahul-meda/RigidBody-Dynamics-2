
#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace Physics
{
	class Collider;

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

	public:
		Body();

		void SetMass(const float m);
		float GetMass() const;

		void SetInvMass(const float m);
		float GetInvMass() const;

		void SetInertia(const glm::mat3& inertia);
		glm::mat3 GetInertia() const;

		void SetDensity(const float d);
		float GetDensity() const;

		void SetRestitution(const float e);
		float GetRestitution() const;

		void SetFriction(const float u);
		float GetFriction() const;

		void SetPosition(const glm::vec3& pos);
		glm::vec3 GetPosition() const;

		void SetOrientation(const glm::quat& o);
		glm::quat GetOrientation() const;

		void SetVelocity(const glm::vec3& vel);
		glm::vec3 GetVelocity() const;

		void SetAngularVelocity(const glm::vec3& w);
		glm::vec3 GetAngularVelocity() const;
		
		const glm::vec3 LocalToGlobalVec(const glm::vec3& v) const;

		const glm::vec3 GlobalToLocalVec(const glm::vec3& v) const;

		const glm::vec3 LocalToGlobalPoint(const glm::vec3& p) const;

		const glm::vec3 GlobalToLocalPoint(const glm::vec3& p) const;

		void AddCollider(Collider* collider);

		// apply force at c.o.m
		void ApplyForce(const glm::vec3& force);

		// apply force at a point on the body (point given in world space)
		void ApplyForce(const glm::vec3& force, const glm::vec3& pt);

		void Update(const float dt);
	};
}