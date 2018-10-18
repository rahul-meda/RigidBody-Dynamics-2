
#pragma once

#include "Collider.h"
#include <vector>
#include "Mesh.h"
#include "Poly.h" 

class HullCollider : public Collider
{
private:
	std::vector<HVertex*> vertices;
	std::vector<HEdge*> edges;
	std::vector<HFace*> faces;

public:
	HullCollider(const HMesh& mesh);

	std::vector<HVertex*>& GetVertices();

	std::vector<HEdge*>& GetEdges();

	std::vector<HFace*>& GetFaces();

	int GetVertexCount() const;
	int GetEdgeCount() const;
	int GetFaceCount() const;

	HVertex* GetVertex(int i) const;
	HEdge* GetEdge(int i) const;
	HFace* GetFace(int i) const;

	// calculaate mass and inertia from geometry data
	void CalculateMass();

	// returns the furthest vertex on the hull in given direction
	int GetSupport(const glm::vec3& dir) const;
};