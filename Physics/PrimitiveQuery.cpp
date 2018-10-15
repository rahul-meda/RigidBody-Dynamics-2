
#include "PrimitiveQuery.h"
#include "HullCollider.h"

bool QueryPoint(Collider* collider, const glm::vec3& point)
{
	switch (collider->GetShape())
	{
	case (Collider::Hull) :
	{
		HullCollider* c = static_cast<HullCollider*>(collider);
		for (int i = 0; i < c->GetFaceCount(); i++)
		{
			return (glm::dot(c->GetFace(i)->edge->tail->position - point, c->GetFace(i)->normal) < 0.0f);
		}
		return true;
	}
	}
}