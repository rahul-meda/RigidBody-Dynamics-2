
#pragma once

#include <glm/glm.hpp>
#include "Model.h"

class Body;

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

class Collider
{
public:
	enum Shape
	{
		Sphere = 1,
		Hull,
		Cylinder,
		Capsule
	};
protected:
	float mass;
	glm::mat3 inertia;

	Shape shape;
	glm::vec3 position;		// stored in owning body's space
	glm::vec3 centroid;		// stored in owning body's space
	glm::vec3 scale;

	Body* body;

	ModelData modelData;
	glm::vec3 color;
public:
	Collider()
		:mass(0),
		inertia(0),
		shape(Hull),
		position(0),
		centroid(0),
		scale(1.0),
		color(0.3, 0.9, 0.3)
	{}

	void SetBody(Body* b);

	float GetMass() const { return mass; };

	glm::mat3 GetInertia() const { return inertia; };

	Shape GetShape() const { return shape; };

	void SetPosition(const glm::vec3& pos) { position = pos; };

	glm::vec3 GetPosition() const { return position; };

	glm::vec3 GetCentroid() const { return centroid; };

	Body* GetBody() const { return body; };

	void SetColor(const glm::vec3& c) { color = c; };

	void SetModelData(const ModelData& m) { modelData = ModelData(m); };

	ModelData GetModelData() const { return modelData; };

	virtual void SetScale(const glm::vec3& scale) {};
};