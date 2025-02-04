#pragma once
#include "delaunay.h"
#include <unordered_map>
#include <unordered_set>

namespace chiori
{
	struct cVEdge
	{
		cVec2 origin;			// origin of the edge
		cVec2 endDir;			// if infinite, endDir is the direction of the edge, otherwise it is the end point
		bool infinite{ false }; // if true, the edge is infinite
		int nextEdge{ -1 };		// index of the next edge in the edge list
		cVEdge(const cVec2& inOrigin = cVec2::zero, const cVec2& inEndDir = cVec2::zero, bool inInfinite = false) :
			origin{ inOrigin }, endDir{ inEndDir }, infinite{ inInfinite } {}
	};

	struct cVCell
	{
		cVec2 site;			// the centriod/site of the cell
		int edgeList{ -1 };	// the index of the latest edge in the cells edge LL
	};

	class cVoronoiDiagram
	{
	public:
		std::vector<cVCell> cells; // Each cell corresponds to a Voronoi region
		std::vector<cVEdge> edges; // All edges in the diagram
		cDelaunayTriangulation delaunay; // Delaunay triangulation

		void CreateVoronoiDiagram(const cVec2* points, unsigned count)
		{
			delaunay.triangulate(points, count);
			
			std::unordered_set<size_t> edgeSet{};
			for (const auto& tri : delaunay.triangles)
			{
				cVec2 circumcenter = tri.circumcenter();
				// Process neighboring triangles
				for (const auto& neighbor : getTriangleNeighbors(tri))
				{
					cVec2 neighborCircumcenter = neighbor.circumcenter();

					// Ensure unique edges
					size_t edgeHash = hashEdge(circumcenter, neighborCircumcenter);
					if (edgeSet.find(edgeHash) == edgeSet.end())
					{
						edges.emplace_back(circumcenter, neighborCircumcenter);
						edgeSet.insert(edgeHash);
					}
				}

				// Handle boundary edges where no neighbor exists
				for (const auto& edge : { cDelEdge(tri.a, tri.b), cDelEdge(tri.b, tri.c), cDelEdge(tri.c, tri.a) })
				{
					if (!hasNeighbor(tri, edge)) // No neighboring triangle => Infinite edge
					{
						cVec2 direction = getPerpendicularBisector(tri, edge);
						edges.emplace_back(circumcenter,  direction, true);
					}
				}
			}
		}
	private:
		cVec2 getPerpendicularBisector(const cDelTriangle& tri, const cDelEdge& edge)
		{
			// Midpoint of the edge
			cVec2 mid = (edge.p1 + edge.p2) * 0.5f;

			// Compute the edge direction and perpendicular vector
			cVec2 edgeDir = (edge.p2 - edge.p1).normalized();
			cVec2 perpDir = cVec2(-edgeDir.y, edgeDir.x); // Rotate by 90 degrees

			// Ensure the bisector points **away** from the triangle
			cVec2 circumcenter = tri.circumcenter();
			cVec2 testDir = circumcenter - mid;
			if (perpDir.dot(testDir) > 0) // Flip direction if it points toward the triangle
			{
				perpDir = -perpDir;
			}

			return perpDir;
		}

		// Retrieves neighboring triangles sharing an edge
		std::vector<cDelTriangle> getTriangleNeighbors(const cDelTriangle& tri)
		{
			std::vector<cDelTriangle> neighbors;
			for (const auto& other : delaunay.triangles)
			{
				if (tri == other) continue;
				if (sharesEdge(tri, other)) neighbors.push_back(other);
			}
			return neighbors;
		}

		// Checks if two triangles share an edge
		bool sharesEdge(const cDelTriangle& t1, const cDelTriangle& t2)
		{
			return (t1.a == t2.a || t1.a == t2.b || t1.a == t2.c) +
				(t1.b == t2.a || t1.b == t2.b || t1.b == t2.c) +
				(t1.c == t2.a || t1.c == t2.b || t1.c == t2.c) == 2;
		}

		// Checks if a triangle has a neighboring triangle along an edge
		bool hasNeighbor(const cDelTriangle& tri, const cDelEdge& edge)
		{
			for (const auto& other : delaunay.triangles)
			{
				if (tri == other) continue;
				if (sharesEdge(tri, other) && (edge == cDelEdge(other.a, other.b) ||
					edge == cDelEdge(other.b, other.c) ||
					edge == cDelEdge(other.c, other.a)))
				{
					return true;
				}
			}
			return false;
		}

		// Hash function for edge uniqueness
		size_t hashEdge(const cVec2& p1, const cVec2& p2) const
		{
			return std::hash<float>()(p1.x) ^ std::hash<float>()(p1.y) ^
				std::hash<float>()(p2.x) ^ std::hash<float>()(p2.y);
		}
	};
}