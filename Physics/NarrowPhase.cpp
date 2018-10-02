
#include "NarrowPhase.h"
#include "Collider.h"
#include "HullCollider.h"
#include "Manifold.h"
#include "Body.h"

namespace Physics
{
	bool QueryFaceAxes(FaceQuery& query, HullCollider* A, HullCollider* B)
	{
		glm::vec3 axis, vA;
		float separation = 0.0f;
		float maxSeparation = FLT_MIN;
		int support, bestFace, bestVertex;

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

	bool IsMinkowskiFace(const glm::vec3& A, const glm::vec3& B, const glm::vec3& B_x_A, const glm::vec3& C, const glm::vec3& D, const glm::vec3& D_x_C)
	{
		// test if the arcs AB and CD intersect on the unit sphere
		float CBA = glm::dot(C, B_x_A);
		float DBA = glm::dot(D, B_x_A);
		float ADC = glm::dot(A, D_x_C);
		float BDC = glm::dot(B, D_x_C);

		return ( (CBA * DBA < 0.0f) &&	// intersection test
				 (ADC * BDC < 0.0f) &&	// intersection test
			     (CBA * BDC > 0.0f) );	// hemisphere test
	}


	bool QueryEdgeAxes(EdgeQuery& query, HullCollider* A, HullCollider* B)
	{
		glm::vec3 edgeA, pA, f1A, f2A;
		glm::vec3 edgeB, pB, f1B, f2B;
		float separation = 0.0f;
		float maxSeparation = FLT_MIN;
		int idA, idB = -1;	// minimum penetrating edges
		glm::vec3 bestAxis;		// minimum penetration axis

		// centroid of A in B's space
		glm::vec3 cA = A->GetCentroid();
		cA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), cA);

		for (int iA = 0; iA < A->GetEdgeCount(); iA++)
		{
			edgeA = A->GetEdge(iA)->GetDirection();
			f1A = A->GetEdge(iA)->face->normal;
			f2A = A->GetEdge(iA)->twin->face->normal;

			for (int iB = 0; iB < B->GetEdgeCount(); iB++)
			{
				edgeB = B->GetEdge(iB)->GetDirection();
				f1B = B->GetEdge(iB)->face->normal;
				f2B = B->GetEdge(iB)->twin->face->normal;

				if (IsMinkowskiFace(f1A, f2A, -edgeA, -f1B, -f2B, -edgeB))
				{
					glm::vec3 axis = glm::cross(edgeA, edgeB);
					float L = glm::length(axis);

					// skip near parallel edges
					const float tolerance = 0.005f;
					if (L < tolerance * glm::sqrt(glm::length2(edgeA) * glm::length2(edgeB)))
						return false;

					pA = A->GetEdge(iA)->tail->position;
					pA = B->GetBody()->LocaltoLocalPoint(A->GetBody(), pA);
					// ensure consistent normal orientation (from A to B)
					if (glm::dot(pA - cA, axis) < 0.0f)
						axis *= -axis;

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
		query.normal = bestAxis;
		query.separation = maxSeparation;

		return true;
	}

	void DetectHullvsHull(std::vector<Manifold>& contacts, HullCollider* A, HullCollider* B)
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
		}
		else
		{
			// face contact
		}
	}

	void DetectCollision(std::vector<Manifold>& contacts, Collider* A, Collider* B)
	{
		switch (A->GetShape())
		{
		case (Collider::Hull) :
		{
			switch (B->GetShape())
			{
			case (Collider::Hull) :
				DetectHullvsHull(contacts, static_cast<HullCollider*>(A), static_cast<HullCollider*>(B));
				break;
			}
		}
		break;
		}
	}
}