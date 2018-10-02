
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

		std::vector<Geometry::HVertex*>& GetVertices();

		std::vector<Geometry::HEdge*>& GetEdges();

		std::vector<Geometry::HFace*>& GetFaces();

		int GetVertexCount() const;
		int GetEdgeCount() const;
		int GetFaceCount() const;

		Geometry::HVertex* GetVertex(int i) const;
		Geometry::HEdge* GetEdge(int i) const;
		Geometry::HFace* GetFace(int i) const;

		void SetScale(const glm::vec3 s);

		// calculaate mass and inertia from geometry data
		void CalculateMass();

		// returns the furthest vertex on the hull in given direction
		int GetSupport(const glm::vec3& dir) const;

		void SetModel(Graphics::Model* model);

		void SetColor(const glm::vec3 color);

		void Render();
	};
}