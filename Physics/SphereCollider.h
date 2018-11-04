
#pragma once

#include "Collider.h"

class SphereCollider : public Collider
{
private:
	float radius;

public:
	SphereCollider(const float radius);

	float GetRadius();

	void CalculateMass();
};