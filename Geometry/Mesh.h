
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Body.h"

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

	HMesh()
	{}

	HMesh(const HMesh& mesh)
	{
		vertices = std::vector<HVertex*>(mesh.vertices.size(), nullptr);
		edges = std::vector<HEdge*>(mesh.edges.size(), nullptr);
		faces = std::vector<HFace*>(mesh.faces.size(), nullptr);

		for (auto mVert : mesh.vertices)
		{
			HVertex* v = new HVertex();
			v->id = mVert->id;
			v->position = mVert->position;
			v->edge = nullptr;
			vertices[mVert->id - 1] = v;
		}

		for (auto mFace : mesh.faces)
		{
			HFace* f = new HFace();
			f->id = mFace->id;
			f->normal = mFace->normal;
			f->edge = nullptr;
			faces[mFace->id - 1] = f;
		}

		for (auto mEdge : mesh.edges)
		{
			HEdge* e = new HEdge();
			e->id = mEdge->id;
			e->duplicate = mEdge->duplicate;
			e->tail = vertices[mEdge->tail->id - 1];
			if (e->tail->edge == nullptr)
				e->tail->edge = e;

			e->face = faces[mEdge->face->id - 1];
			edges[mEdge->id - 1] = e;
		}

		for (auto mEdge : mesh.edges)
		{
			edges[mEdge->id - 1]->next = edges[mEdge->next->id - 1];
			if (mEdge->twin != nullptr)
			{
				edges[mEdge->id - 1]->twin = edges[mEdge->twin->id - 1];
			}
		}

		for (auto mFace : mesh.faces)
		{
			faces[mFace->id - 1]->edge = edges[mFace->edge->id - 1];
			faces[mFace->id - 1]->normal = glm::cross(faces[mFace->id - 1]->edge->GetDirection(), faces[mFace->id - 1]->edge->next->GetDirection());
			faces[mFace->id - 1]->normal = glm::normalize(faces[mFace->id - 1]->normal);
		}

		// arrange edges such that first half are edges, 
		// and next half are twins
		std::vector<HEdge*> e1, e2;
		for (HEdge* e : edges)
		{
			if (!(e->duplicate))
				e1.push_back(e);
			else
				e2.push_back(e);
		}
		edges.clear();
		for (HEdge* e : e1)
			edges.push_back(e);
		for (HEdge* e : e2)
			edges.push_back(e);
	}

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

	void GetModelData(ModelData& m) const
	{
		m.vertices.clear();
		m.indices.clear();
		m.frameIndices.clear();

		for (auto v : vertices)
		{
			m.vertices.push_back(v->position);
		}
		GetTriangleIndices(m.indices);
		GetLineIndices(m.frameIndices);
	}
};