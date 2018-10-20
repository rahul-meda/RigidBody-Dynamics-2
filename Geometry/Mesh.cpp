
#include "Mesh.h"

glm::vec3 HEdge::GetDirection() const
{
	return next->tail->position - tail->position;
}

HEdge* HEdge::Prev()
{
	int n = 0;
	HEdge* e = face->edge;
	do {
		n++;
		e = e->next;
	} while (e != face->edge);

	HEdge* prev = this;
	for (int i = 1; i < n; i++)
		prev = prev->next;

	return prev;
}

void HFace::CalculateNormal()
{
	normal = glm::cross(edge->GetDirection(), edge->next->GetDirection());
	normal = glm::normalize(normal);
}

HMesh::HMesh()
{}

HMesh::HMesh(const HMesh& mesh)
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

void HMesh::RemoveObselete()
{
	std::vector<HVertex*> tempV;
	std::vector<HEdge*> tempE;
	std::vector<HFace*> tempF;

	bool remove = false;
	int id = 0;
	for (auto v : vertices)
	{
		for (auto i : vids)
		{
			if (v->id == i)
				remove = true;
		}
		if (!remove)
		{
			v->id = ++id;
			tempV.push_back(v);
		}
		remove = false;
	}
	vertices = tempV;

	remove = false;
	id = 0;
	for (auto e : edges)
	{
		for (auto i : eids)
		{
			if (e->id == i)
				remove = true;
		}
		if (!remove)
		{
			e->id = ++id;
			tempE.push_back(e);
		}
		remove = false;
	}
	edges = tempE;

	remove = false;
	id = 0;
	for (auto f : faces)
	{
		for (auto i : fids)
		{
			if (f->id == i)
				remove = true;
		}
		if (!remove)
		{
			f->id = ++id;
			tempF.push_back(f);
		}
		remove = false;
	}
	faces = tempF;

	vids.clear();
	eids.clear();
	fids.clear();
}

bool HMesh::AreCoplanar(HFace* f1, HFace* f2)
{
	const static float e = 0.01;
	float diff = 1.0f - glm::dot(f1->normal, f2->normal);

	return (diff == 0.0f || diff < e);
}

void HMesh::FixTopological(HEdge* in, HEdge* out)
{
	HFace* adjFace;
	int nVerts = 0;
	if (in->twin->face == out->twin->face)
	{
		adjFace = in->twin->face;
		HEdge* e = adjFace->edge;
		do {
			nVerts++;
			e = e->next;
		} while (e != adjFace->edge);

		if (nVerts == 3)
		{
			HEdge* third = in->twin->next;
			out->face->edge = out->next;
			third->face = in->face;
			HEdge* inPrev = in->Prev();
			inPrev->next = third;
			third->next = out->next;

			fids.push_back(adjFace->id);
			eids.push_back(in->id);
			eids.push_back(in->twin->id);
			eids.push_back(out->id);
			eids.push_back(out->twin->id);
			vids.push_back(out->tail->id);
		}
		else
		{
			out->face->edge = out->next;
			in->twin->tail = out->twin->tail;
			in->next = out->next;

			adjFace->edge = in->twin->next;
			HEdge* outPrev = out->twin->Prev();
			outPrev->next = in->twin;
			outPrev->next->twin = in;

			eids.push_back(out->id);
			eids.push_back(out->twin->id);
			vids.push_back(out->tail->id);
		}
	}
}

int HMesh::MergeFaces()
{
	int n = 0;
	HEdge* e;
	for (int i = 0; i < edges.size() / 2; i++)
	{
		e = edges[i];
		if (AreCoplanar(e->face, e->twin->face))
		{
			n++;
			// assign edge reference for the merged face
			e->face->edge = e->next;

			// assign edges of the face to be merged
			int nEdges = 0;
			HEdge* curr = e->twin->face->edge;
			do {
				nEdges++;
				curr = curr->next;
			} while (curr != e->twin->face->edge);

			curr = e->twin->next;
			for (int i = 1; i < nEdges; i++)
			{
				curr->face = e->face;
				curr = curr->next;
			}

			// connect in-coming and out-going extreme edges
			HEdge* edgePrev = e->Prev();
			HEdge* twinPrev = e->twin->Prev();
			edgePrev->next = e->twin->next;
			twinPrev->next = e->next;

			fids.push_back(e->twin->face->id);
			eids.push_back(e->id);
			eids.push_back(e->twin->id);

			// check if topological invariants are violated while merging 
			// each vertex must have at least 3 adjacent faces
			// ToDo : 1. Save prev edge for each Hedge, 2. Recalculate normals - Newell planes
			//FixTopological(edgePrev, edgePrev->next);
			//FixTopological(twinPrev, twinPrev->next);
		}
	}

	RemoveObselete();

	return n;
}

void HMesh::GetTriangleIndices(std::vector<int>& indices) const
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

void HMesh::GetLineIndices(std::vector<int>& indices) const
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

void HMesh::GetModelData(ModelData& m) const
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