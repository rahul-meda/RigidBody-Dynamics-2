
#include "Body.h"
#include "HullCollider.h"
#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>

namespace Physics
{
	HullCollider::HullCollider(const Geometry::HMesh& mesh)
	{
		shape = Hull;
		vertices = mesh.vertices;
		edges = mesh.edges;
		faces = mesh.faces;
	}

	void HullCollider::CalculateMass()
	{
		glm::vec3 diag(0.0f);
		glm::vec3 offDiag(0.0f);
		float volume = 0.0f;
		glm::vec3 localCentroid;

		for (int i = 0; i < faces.size(); i++)
		{
			auto start = faces[i]->edge;
			auto middle = start->next;
			auto last = middle->next;

			while (start != last)
			{
				glm::vec3 u = start->tail->position;
				glm::vec3 v = middle->tail->position;
				glm::vec3 w = last->tail->position;

				float currentVolume = glm::dot(u, glm::cross(v, w));
				volume += currentVolume;
				localCentroid += (u + v + w) * currentVolume;

				for (int j = 0; j < 3; ++j)
				{
					int j1 = (j + 1) % 3;
					int j2 = (j + 2) % 3;

					diag[j] += (
						u[j] * v[j] + v[j] * w[j] + w[j] * u[j] +
						u[j] * u[j] + v[j] * v[j] + w[j] * w[j]) * currentVolume;

					offDiag[j] += (
						u[j1] * v[j2] + v[j1] * w[j2] + w[j1] * u[j2] +
						u[j1] * w[j2] + v[j1] * u[j2] + w[j1] * v[j2] +
						u[j1] * u[j2] * 2.0f + v[j1] * v[j2] * 2.0f + w[j1] * w[j2] * 2.0f) * currentVolume;
				}

				middle = last;
				last = last->next;
			}
		}

		localCentroid /= (volume * 4.0f);
		glm::vec3 globalCentroid = body->LocalToGlobalPoint(position) + body->LocalToGlobalVec(localCentroid);
		centroid = body->GlobalToLocalPoint(globalCentroid);

		volume *= (1.0f / 6.0f);
		diag /= volume * 60.0f;
		offDiag /= volume * 120.0f;
		mass = body->GetDensity() * volume;

		inertia = glm::mat3(diag.y + diag.z, -offDiag.z     , -offDiag.y,
							-offDiag.z     , diag.x + diag.z, -offDiag.x,
							-offDiag.y     , -offDiag.x     , diag.x + diag.y);

		inertia *= mass;
	}

	void HullCollider::SetScale(const glm::vec3 s)
	{
		scale = s;
		for (auto v : vertices)
		{
			v->position.x *= scale.x;
			v->position.y *= scale.y;
			v->position.z *= scale.z;
		}
	}

	void HullCollider::SetModel(Graphics::Model* model)
	{
		poly = static_cast<Graphics::Poly*>(model);
	}

	void HullCollider::Render()
	{
		static glm::mat4 T(1), R(1), S(1), M(1), V(1), P(1), MVP(1);
		V = Graphics::Camera::GetInstance().GetViewMatrix();
		P = Graphics::Camera::GetInstance().GetProjectionMatrix();

		T = glm::translate(body->LocalToGlobalPoint(position));
		R = glm::toMat4(body->GetOrientation());
		S = glm::scale(scale);
		M = T * R * S;
		MVP = P * V * M;
		poly->SetMVP(MVP);
		poly->SetColor(color);
		poly->Render();

		poly->GetFrame()->SetMVP(MVP);
		poly->GetFrame()->Render();
	}
}