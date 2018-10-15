
#include "Body.h"
#include "HullCollider.h"
#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "Line.h"

HullCollider::HullCollider(const HMesh& mesh)
{
	shape = Hull;

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

std::vector<HVertex*>& HullCollider::GetVertices()
{
	return vertices;
}

std::vector<HEdge*>& HullCollider::GetEdges()
{
	return edges;
}

std::vector<HFace*>& HullCollider::GetFaces()
{
	return faces;
}

int HullCollider::GetVertexCount() const
{
	return vertices.size();
}

int HullCollider::GetEdgeCount() const
{
	return edges.size() / 2;
}

int HullCollider::GetFaceCount() const
{
	return faces.size();
}

HVertex* HullCollider::GetVertex(int i) const
{
	return vertices[i];
}

HEdge* HullCollider::GetEdge(int i) const
{
	return edges[i];
}

HFace* HullCollider::GetFace(int i) const
{
	return faces[i];
}

void HullCollider::SetScale(const glm::vec3& s)
{
	scale = s;
	for (auto v : vertices)
	{
		v->position.x *= scale.x;
		v->position.y *= scale.y;
		v->position.z *= scale.z;
	}
	for (glm::vec3& v : modelData.vertices)
	{
		v.x *= s.x;
		v.y *= s.y;
		v.z *= s.z;
	}
}

void HullCollider::CalculateMass()
{
	glm::vec3 diag(0.0f);
	glm::vec3 offDiag(0.0f);
	float volume = 0.0f;
	glm::vec3 localCentroid(0.0);

	for (int i = 0; i < faces.size(); i++)
	{
		auto start = faces[i]->edge;
		auto middle = start->next;
		auto last = middle->next;

		while (start != last)
		{
			glm::vec3 u = start->tail->position;
			glm::vec3 v = middle->tail->position;
			glm::vec3 w = last->tail->position;

			float currentVolume = glm::dot(u, glm::cross(v, w));
			volume += currentVolume;
			localCentroid += (u + v + w) * currentVolume;

			for (int j = 0; j < 3; ++j)
			{
				int j1 = (j + 1) % 3;
				int j2 = (j + 2) % 3;

				diag[j] += (
					u[j] * v[j] + v[j] * w[j] + w[j] * u[j] +
					u[j] * u[j] + v[j] * v[j] + w[j] * w[j]) * currentVolume;

				offDiag[j] += (
					u[j1] * v[j2] + v[j1] * w[j2] + w[j1] * u[j2] +
					u[j1] * w[j2] + v[j1] * u[j2] + w[j1] * v[j2] +
					u[j1] * u[j2] * 2.0f + v[j1] * v[j2] * 2.0f + w[j1] * w[j2] * 2.0f) * currentVolume;
			}

			middle = last;
			last = last->next;
		}
	}

	localCentroid /= (volume * 4.0f);
	glm::vec3 globalCentroid = body->LocalToGlobalPoint(position) + body->LocalToGlobalVec(localCentroid);
	centroid = body->GlobalToLocalPoint(globalCentroid);

	volume *= (1.0f / 6.0f);
	diag /= volume * 60.0f;
	offDiag /= volume * 120.0f;
	mass = body->GetDensity() * volume;

	inertia = glm::mat3(diag.y + diag.z, -offDiag.z, -offDiag.y,
		-offDiag.z, diag.x + diag.z, -offDiag.x,
		-offDiag.y, -offDiag.x, diag.x + diag.y);

	inertia *= mass;
}

// To Do : Hill Climbing
int HullCollider::GetSupport(const glm::vec3& dir) const
{
	float dot = 0.0f;
	float maxDot = -FLT_MAX;
	int index = -1;

	for (int i = 0; i < vertices.size(); i++)
	{
		dot = glm::dot(vertices[i]->position, dir);
		if (dot > maxDot)
		{
			maxDot = dot;
			index = i;
		}
	}

	return index;
}

void HullCollider::SetModel(Model* model)
{
	poly = static_cast<Poly*>(model);
}

void HullCollider::Render()
{
	static glm::mat4 T(1), R(1), S(1), M(1), V(1), P(1), VP(1), MVP(1);
	V = Camera::GetInstance().GetViewMatrix();
	P = Camera::GetInstance().GetProjectionMatrix();

	T = glm::translate(body->LocalToGlobalPoint(position));
	R = glm::toMat4(body->GetOrientation());
	S = glm::scale(scale);
	M = T * R * S;
	VP = P * V;
	MVP = VP * M;
	poly->SetMVP(MVP);
	poly->SetColor(color);
	poly->Render();

	poly->GetFrame()->SetMVP(MVP);
	poly->GetFrame()->Render();

	// face normals
	/*for (HFace* f : faces)
	{
	glm::vec3 c(0);
	int n = 0;

	HEdge* e = f->edge;
	do {
	c += e->tail->position;
	n++;
	e = e->next;
	} while (e != f->edge);
	c /= n;
	c = body->LocalToGlobalPoint(c);

	std::vector<glm::vec3> verts = { c, c + 2.0f*body->LocalToGlobalVec(f->normal) };
	std::vector<int> ids = {0, 1};
	Line* line = new Line(verts, ids);
	line->SetMVP(VP);
	line->SetColor(glm::vec3(0,1,0));
	line->Render();
	delete line;
	}*/
}