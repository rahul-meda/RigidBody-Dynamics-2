
#include "Body.h"
#include "HullCollider.h"
#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include "Line.h"

HullCollider::HullCollider(const HMesh& mesh)
{
	shape = Hull;

	vertices = mesh.vertices;
	edges = mesh.edges;
	faces = mesh.faces;
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

// calculates the triangle indices
// used by gaphics to render solid mesh in triangle mode
void HullCollider::GetTriangleIndices(std::vector<int>& indices) const
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
void HullCollider::GetLineIndices(std::vector<int>& indices) const
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

void HullCollider::GetModelData(ModelData& m) const
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
	/*glm::vec3 globalCentroid = body->LocalToGlobalPoint(position) + body->LocalToGlobalVec(localCentroid);
	centroid = body->GlobalToLocalPoint(globalCentroid);*/
	centroid = localCentroid;

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