
#pragma once

#include <glm/glm.hpp>
#include <vector>


// half-edge data structure to represent mesh collision geometry
// for fast adjacency queries

struct HEdge;
struct HFace;

// represents a vertex on the mesh
struct HVertex
{
	glm::vec3 position;		// vertex coordinate position
	HEdge* edge;			// the edge pointing to this vertex
	int id;					// vertex index

	bool dup = false;
};

// represents an edge on the mesh
struct HEdge
{
	HVertex* tail;			// vertex pointing to this edge
	HFace* face;			// face to the left of this edge (CCW winding)
	HEdge* next;			// next edge belonging to the same face
	HEdge* twin;			// opposite edge pointing to the tail
	int id;					// edge index
	bool duplicate;			// marked false for original edge, and true for the twin

	glm::vec3 GetDirection() const
	{
		return next->tail->position - tail->position;
	}
};

// represents a face on the mesh
struct HFace
{
	HEdge* edge;			// any edge on this face
	glm::vec3 normal;		//face normal should be pointing outwards;
	int id;					// face index

	void CalculateNormal()
	{
		normal = glm::cross(edge->GetDirection(), edge->next->GetDirection());
	}
};

struct HMesh
{
	std::vector<HVertex*> vertices;
	std::vector<HEdge*> edges;
	std::vector<HFace*> faces;

	// calculates the triangle indices
	// used by gaphics to render solid mesh in triangle mode
	void GetTriangleIndices(std::vector<int>& indices) const
	{
		for (HFace* f : faces)
		{
			HEdge* start = f->edge;
			HEdge* e = start;
			int first, second, third;
			first = e->tail->id - 1;
			do		// triangle fan
			{
				second = e->next->tail->id - 1;
				e = e->next;
				third = e->next->tail->id - 1;

				indices.push_back(first);
				indices.push_back(second);
				indices.push_back(third);
			} while (e->next->next != start);
		}
	}

	// To do
	// calculates line indices
	// used by graphics to render mesh in wire-frame mode
	void GetLineIndices(std::vector<int>& indices) const
	{
		int id1, id2;
		for (int i = 0; i < edges.size() / 2; i++)
		{
			id1 = edges[i]->tail->id - 1;
			id2 = edges[i]->next->tail->id - 1;

			indices.push_back(id1);
			indices.push_back(id2);
		}
	}
};