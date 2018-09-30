
#pragma once

#include "Collider.h"
#include <vector>
#include "Mesh.h"
#include "Poly.h" 

namespace Physics
{
	class HullCollider : public Collider
	{
	private:
		std::vector<Geometry::HVertex*> vertices;
		std::vector<Geometry::HEdge*> edges;
		std::vector<Geometry::HFace*> faces;

		Graphics::Poly* poly;
		Graphics::Poly* frame;

	public:
		HullCollider(const Geometry::HMesh& mesh);

		void CalculateMass();

		void SetScale(const glm::vec3 s);

		void SetModel(Graphics::Model* model);

		void SetColor(const glm::vec3 color);

		void Render();
	};
}