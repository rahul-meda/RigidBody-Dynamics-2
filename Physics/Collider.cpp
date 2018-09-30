
#include "Body.h"
#include "Collider.h"

namespace Physics
{
	void Collider::SetBody(Body* b)
	{
		body = b;
		position = body->GlobalToLocalPoint(position);
	}
}