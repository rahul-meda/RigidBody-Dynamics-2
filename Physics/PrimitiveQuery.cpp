
#include "PrimitiveQuery.h"
#include "HullCollider.h"
#include "Body.h"

bool QueryPoint(Collider* collider, const glm::vec3& point)
{
	switch (collider->GetShape())
	{
	case (Collider::Hull) :
	{
		HullCollider* c = static_cast<HullCollider*>(collider);
		for (int i = 0; i < c->GetFaceCount(); i++)
		{
			glm::vec3 P = c->GetFace(i)->edge->tail->position;
			P = c->GetBody()->LocalToGlobalPoint(P);
			glm::vec3 n = c->GetFace(i)->normal;
			n = c->GetBody()->LocalToGlobalVec(n);
			if (glm::dot(P - point, n) < 0.0f)
				return false;
		}
		return true;
	}
	}
}