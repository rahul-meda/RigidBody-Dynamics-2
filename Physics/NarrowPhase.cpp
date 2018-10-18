
#include "NarrowPhase.h"
#include "Collider.h"
#include "HullCollider.h"
#include "Contact.h"
#include "Body.h"
#include "Geometry.h"

#define EPSILON 0.005

bool QueryFaceAxes(FaceQuery& query, HullCollider* A, HullCollider* B)
{
	glm::vec3 axis, vA;
	float separation = 0.0f;
	float maxSeparation = -FLT_MAX;
	int support, bestFace = -1, bestVertex = -1;

	for (int i = 0; i < A->GetFaceCount(); i++)
	{
		axis = A->GetFace(i)->normal;
		axis = B->GetBody()->LocalToLocalVec(A->GetBody(), axis);

		support = B->GetSupport(-axis);				// max projection of B on axis

		vA = A->GetFace(i)->edge->tail->position;			// min prohection of A on axis
		vA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), vA);

		separation = glm::dot(B->GetVertex(support)->position - vA, axis);	// compare projections

		if (separation > 0.0f)						// early out?
			return false;

		if (separation > maxSeparation)
		{
			maxSeparation = separation;
			bestFace = i;
			bestVertex = support;
		}
	}
	query.faceIndex = bestFace;
	query.separation = maxSeparation;
	query.vertIndex = bestVertex;

	return true;
}

bool QueryEdgeAxes(EdgeQuery& query, HullCollider* A, HullCollider* B)
{
	glm::vec3 edgeA, pA, f1A, f2A;
	glm::vec3 edgeB, pB, f1B, f2B;
	float separation = 0.0f;
	float maxSeparation = -FLT_MAX;
	int idA = -1, idB = -1;	// minimum penetrating edges
	glm::vec3 bestAxis;		// minimum penetration axis

	// centroid of A in B's space
	glm::vec3 cA = A->GetCentroid();
	cA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), cA);

	for (int iA = 0; iA < A->GetEdgeCount(); iA++)
	{
		edgeA = A->GetEdge(iA)->GetDirection();
		edgeA = B->GetBody()->LocalToLocalVec(A->GetBody(), edgeA);
		f1A = A->GetEdge(iA)->face->normal;
		f1A = B->GetBody()->LocalToLocalVec(A->GetBody(), f1A);
		f2A = A->GetEdge(iA)->twin->face->normal;
		f2A = B->GetBody()->LocalToLocalVec(A->GetBody(), f2A);

		for (int iB = 0; iB < B->GetEdgeCount(); iB++)
		{
			edgeB = B->GetEdge(iB)->GetDirection();
			f1B = B->GetEdge(iB)->face->normal;
			f2B = B->GetEdge(iB)->twin->face->normal;

			if (IsMinkowskiFace(f1A, f2A, -edgeA, -f1B, -f2B, -edgeB))
			{
				glm::vec3 axis = glm::cross(edgeA, edgeB);
				float L = glm::length(axis);

				glm::vec3 v(10, 0, 0);
				float len = glm::length(v);
				v = glm::normalize(v);
				glm::vec3 vn = v / len;

				// skip near parallel edges
				const float tolerance = 0.005f;
				if (L < tolerance * glm::sqrt(glm::length2(edgeA) * glm::length2(edgeB)))
					continue;

				pA = A->GetEdge(iA)->tail->position;
				pA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), pA);
				// ensure consistent normal orientation (from A to B)
				if (glm::dot(pA - cA, axis) < 0.0f)
					axis = -axis;

				axis /= L;

				pB = B->GetEdge(iB)->tail->position;
				separation = glm::dot(pB - pA, axis);

				if (separation > 0.0f)	return false;

				if (separation > maxSeparation)
				{
					maxSeparation = separation;
					idA = iA;
					idB = iB;
					bestAxis = axis;
				}
			}
		}
	}
	query.edgeIndex1 = idA;
	query.edgeIndex2 = idB;
	query.normal = B->GetBody()->LocalToGlobalVec(bestAxis);
	query.separation = maxSeparation;

	return true;
}

bool IsMinkowskiFace(const glm::vec3& A, const glm::vec3& B, const glm::vec3& B_x_A, const glm::vec3& C, const glm::vec3& D, const glm::vec3& D_x_C)
{
	// test if the arcs AB and CD intersect on the unit sphere
	float CBA = glm::dot(C, B_x_A);
	float DBA = glm::dot(D, B_x_A);
	float ADC = glm::dot(A, D_x_C);
	float BDC = glm::dot(B, D_x_C);

	return ((CBA * DBA < 0.0f) &&	// intersection test
		(ADC * BDC < 0.0f) &&	// intersection test
		(CBA * BDC > 0.0f));	// hemisphere test
}

void CreateEdgeContact(std::vector<Manifold>& manifolds, HullCollider* A, HullCollider*B, const EdgeQuery& query)
{
	HEdge* edgeA = A->GetEdge(query.edgeIndex1);
	HEdge* edgeB = B->GetEdge(query.edgeIndex2);
	glm::vec3 pA = A->GetBody()->LocalToGlobalPoint(edgeA->tail->position);
	glm::vec3 pB = B->GetBody()->LocalToGlobalPoint(edgeB->tail->position);
	glm::vec3 d1 = A->GetBody()->LocalToGlobalVec(edgeA->GetDirection());
	glm::vec3 d2 = B->GetBody()->LocalToGlobalVec(edgeB->GetDirection());
	glm::vec3 r = pA - pB;

	// Parameters to solve segment-segment overlap
	// To do: can be converted to 2D segment overlap by
	// projecting edgeB's end points on to the plane
	// passing through edgeA, and normal to both edges
	float a, b, c, d, e, f, s;
	a = glm::dot(d1, d1);
	b = glm::dot(d1, d2);
	c = glm::dot(d1, r);
	e = glm::dot(d2, d2);
	f = glm::dot(d2, r);

	d = a*e - b*b;
	assert(d > 0.0f);

	s = (b*f - c*e) / d;
	assert((s > 0.0f) && (s < 1.0f));

	glm::vec3 pos = pA + s*d1;

	Contact contact(A->GetBody(), B->GetBody(), pos, query.normal, -query.separation);
	Manifold m;
	m.contacts.push_back(contact);
	manifolds.push_back(m);
}

void CreateFaceContact(std::vector<Manifold>& manifolds, HullCollider* incident, HullCollider* reference, int incidentFace, int referenceFace)
{
	std::vector<glm::vec3> inPoly;			// the face to be clipped - vertices on reference face
	std::vector<HalfSpace> planes;	// the clipping planes - faces represented by each edge on the incident face

	// extract the vertices from incident face - in world space
	auto start = incident->GetFace(incidentFace)->edge;
	auto edge = start;
	do {
		inPoly.push_back(incident->GetBody()->LocalToGlobalPoint(edge->tail->position));
		edge = edge->next;
	} while (edge != start);

	{
		// extract the faces representing the clipping planes - in world space
		glm::vec3 normal, point;
		start = reference->GetFace(referenceFace)->edge;
		edge = start;
		do {
			normal = reference->GetBody()->LocalToGlobalVec(edge->twin->face->normal);
			point = reference->GetBody()->LocalToGlobalPoint(edge->tail->position);
			planes.push_back(HalfSpace(normal, point));
			edge = edge->next;
		} while (edge != start);
	}

	std::vector<glm::vec3> outPoly;		// the clipped poly
	glm::vec3 A, B;
	float dA, dB;

	for (auto plane : planes)
	{
		A = inPoly[inPoly.size() - 1];
		for (int i = 0; i < inPoly.size(); i++)
		{
			B = inPoly[i];
			dA = glm::dot(A, plane.normal) - plane.distance;
			dB = glm::dot(B, plane.normal) - plane.distance;

			// A and B Behind or On plane - out B
			if ((dA < 0.0f && dB < 0.0f) || (glm::abs(dA) < EPSILON) || (glm::abs(dB) < EPSILON))
			{
				outPoly.push_back(B);
			}
			// A Infront and B Behind - out I, B
			else if (dA > 0.0f && dB < 0.0f)
			{
				outPoly.push_back(A + (dA / (dA - dB)) * (B - A));
				outPoly.push_back(B);
			}
			// A Behind and B Infront - out I
			else if (dA < 0.0f && dB > 0.0f)
			{
				outPoly.push_back(A + (dA / (dA - dB)) * (B - A));
			}
			A = B;
		}
		inPoly = outPoly;
		outPoly.clear();
	}

	// To Do : reduce clipped vertices size to <= 4

	glm::vec3 normal = reference->GetFace(referenceFace)->normal;
	normal = reference->GetBody()->LocalToGlobalVec(normal);
	glm::vec3 point = reference->GetFace(referenceFace)->edge->tail->position;
	point = reference->GetBody()->LocalToGlobalPoint(point);
	HalfSpace refFace(normal, point);
	float distance;
	int count = 0;
	Manifold m;

	for (glm::vec3 v : inPoly)
	{
		distance = refFace.Distance(v);
		if (distance <= EPSILON)
		{
			Contact contact(reference->GetBody(), incident->GetBody(), v, normal, -distance);
			m.contacts.push_back(contact);
		}
	}

	if (m.contacts.size() > 0)
	manifolds.push_back(m);
}

int FindIncidentFace(HullCollider* incident, HullCollider* reference, int referenceFace)
{
	int incidentFace;
	float dot;
	float minDot = FLT_MAX;
	glm::vec3 refNormal = reference->GetFace(referenceFace)->normal;
	refNormal = incident->GetBody()->LocalToLocalVec(reference->GetBody(), refNormal);

	for (int i = 0; i < incident->GetFaceCount(); i++)
	{
		dot = glm::dot(incident->GetFace(i)->normal, refNormal);
		if (dot < minDot)
		{
			minDot = dot;
			incidentFace = i;
		}
	}
	return incidentFace;
}

void DetectHullvsHull(std::vector<Manifold>& manifolds, HullCollider* A, HullCollider* B)
{
	FaceQuery faceQueryA;
	if (!QueryFaceAxes(faceQueryA, A, B))
		return;
	assert(faceQueryA.separation <= 0.0f);

	FaceQuery faceQueryB;
	if (!QueryFaceAxes(faceQueryB, B, A))
		return;
	assert(faceQueryB.separation <= 0.0f);

	EdgeQuery edgeQuery;
	if (!QueryEdgeAxes(edgeQuery, A, B))
		return;
	assert(edgeQuery.separation <= 0.0f);

	float maxFaceSep = faceQueryA.separation > faceQueryB.separation ? faceQueryA.separation : faceQueryB.separation;
	float epsilon = 0.05f;

	if (edgeQuery.separation > maxFaceSep + epsilon)
	{
		// edge contact
		CreateEdgeContact(manifolds, A, B, edgeQuery);
	}
	else
	{
		// face contact
		int incidentFace;
		if (faceQueryA.separation > faceQueryB.separation + epsilon)
		{
			incidentFace = FindIncidentFace(B, A, faceQueryA.faceIndex);
			CreateFaceContact(manifolds, B, A, incidentFace, faceQueryA.faceIndex);
		}
		else
		{
			incidentFace = FindIncidentFace(A, B, faceQueryB.faceIndex);
			CreateFaceContact(manifolds, A, B, incidentFace, faceQueryB.faceIndex);
		}
	}
}

void DetectCollision(std::vector<Manifold>& manifolds, Collider* A, Collider* B)
{
	switch (A->GetShape())
	{
	case (Collider::Hull) :
	{
		switch (B->GetShape())
		{
		case (Collider::Hull) :
			DetectHullvsHull(manifolds, static_cast<HullCollider*>(A), static_cast<HullCollider*>(B));
			break;
		}
	}
	break;
	}
}