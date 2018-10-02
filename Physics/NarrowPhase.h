
#pragma once

#include <vector>

namespace Physics
{
	// detect collision b/w colliders
	// and generate detailed collision data

	class Manifold;
	class Collider;
	class HullCollider;

	// Collision info for nearest face features b/w 2 colliders A and B
	struct FaceQuery
	{
		float separation;	// negative values indicate point of B inside face of A

		int faceIndex;		// index of face of collider A

		int vertIndex;		// index of best vertex of collider B (min penetrating vertex)
	};

	// Collision info for nearest edge features b/w 2 colliders A and B
	struct EdgeQuery
	{
		float separation;	// negative values indicate overlap b/w edges

		int edgeIndex1;		// indices of edges of A and B
		int edgeIndex2;

		glm::vec3 normal;	// parpendicular to both edges, and pointing from A to B
	};

	// SAT
	// Project hulls onto every possible separating axis
	// and check for overlap of projections

	// Axes are face normals of A
	// Retruns true if there is overlap on every face axis
	// Store minimizing(least penetrating) vertex of B inside A, and assosiated face of A
	bool QueryFaceAxes(FaceQuery& query, HullCollider* A, HullCollider* B);

	// Axes are all possible cross products of edge combinations (edgeA, edgeB)
	// Edge Pruing - Guass Map Optimization - Dirk Gregorius - GDC 2013
	// The possible separating axes are the edges that form a face on the Minkowski difference (A-B)
	bool QueryEdgeAxes(EdgeQuery& query, HullCollider*A, HullCollider* B);

	// @params A, B, C, D are faces of hull that represent points on the Gauss Map
	// The arcs connecting the faces represent the edges
	// Two edges form a face on the Minkowski difference if their associated arcs intersect
	// A-B is arc1(edge from hull A) and C-D is arc2(edge from hull B)
	bool IsMinkowskiFace(const glm::vec3& A, const glm::vec3& B, const glm::vec3& B_x_A, const glm::vec3& C, const glm::vec3& D, const glm::vec3& D_x_C);

	void DetectHullvsHull(std::vector<Manifold>& contacts, HullCollider* A, HullCollider* B);

	void DetectCollision(std::vector<Manifold>& contacts, Collider* A, Collider* B);
}