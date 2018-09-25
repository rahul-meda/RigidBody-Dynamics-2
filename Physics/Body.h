
#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Model.h"

namespace Physics
{
	class Body
	{
	private:
		float invMass;
		glm::mat3 localInvInertia;
		glm::mat3 invInertia;

		glm::vec3 position;
		glm::quat orientation;
		glm::mat3 R;

		glm::vec3 velocity;
		glm::vec3 angularVelocity;

		glm::vec3 forceSum;
		glm::vec3 torqueSum;

		Graphics::Model* model;

	public:
		Body();

		void SetMass(const float m);
		float GetMass() const;

		void SetInvMass(const float m);
		float GetInvMass() const;

		void SetInertia(const glm::mat3& inertia);
		glm::mat3 GetInertia() const;

		void SetPosition(const glm::vec3 pos);
		glm::vec3 GetPosition() const;

		void SetOrientation(const glm::quat o);
		glm::quat GetOrientation() const;

		void SetVelocity(const glm::vec3 vel);
		glm::vec3 GetVelocity() const;

		void SetAngularVelocity(const glm::vec3 w);
		glm::vec3 GetAngularVelocity() const;

		void SetModel(Graphics::Model* m);
		Graphics::Model* GetModel() const;

		void Update(const float dt);
	};
}