
#include "Body.h"
#include "Collider.h"

void Collider::SetBody(Body* b)
{
	body = b;
	position = body->GlobalToLocalPoint(position);
}