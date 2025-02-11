#pragma once
#include "chioriMath.h"
#include "chioriAllocator.h"
#include <vector>


// TODO: Optimize and convert to using allocators

namespace chiori
{
	struct cDelTriangle
	{
		cDelTriangle(const cVec2& inA = cVec2::zero, const cVec2& inB = cVec2::zero, const cVec2& inC = cVec2::zero) :
			a{ inA }, b{ inB }, c{ inC } {}

		cVec2 circumcenter() const
		{
			float dA = a.dot(a);
			float dB = b.dot(b);
			float dC = c.dot(c);
			float det = 2.0f * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
			return { (dA * (b.y - c.y) + dB * (c.y - a.y) + dC * (a.y - b.y)) / det,
								 (dA * (c.x - b.x) + dB * (a.x - c.x) + dC * (b.x - a.x)) / det };
		}
		bool circumcircleContains(const cVec2& p) const
		{
			const cVec2 center = circumcenter();
			float radiusSqr = distanceSqr(a, center);
			float distSqr = distanceSqr(p, center);
			return distSqr <= radiusSqr;
		}
		
		bool operator==(const cDelTriangle& other) const
		{
			// handle cyclic ordering
			return	(a == other.a && b == other.b && c == other.c) ||
					(a == other.b && b == other.c && c == other.a) ||
					(a == other.c && b == other.a && c == other.b);
		}

		cVec2 a, b, c;
	};

	struct cDelEdge
	{
		cDelEdge(const cVec2& inP1 = cVec2::zero, const cVec2& inP2 = cVec2::zero) :
			p1{ inP1 }, p2{ inP2 } {}

		bool operator==(const cDelEdge& other) const {
			return (p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1);
		}

		cVec2 p1, p2;
	};

	class cDelaunayTriangulation
	{		
	public:
		std::vector<cDelTriangle> triangles{};

		void triangulate(const cVec2* points, unsigned inCount)
		{
			// check for valid input
			cassert(points);
			cassert(inCount > 0);
			// Create bounding 'super' triangle
			// Find the maximum and minimum vertex bounds.
			// This is to allow calculation of the bounding triangle
			float xmin = points[0].x, xmax = xmin;
			float ymin = points[0].y, ymax = ymin;
			for (unsigned i = 1; i < inCount; ++i) {
				if (points[i].x < xmin) xmin = points[i].x;
				if (points[i].x > xmax) xmax = points[i].x;
				if (points[i].y < ymin) ymin = points[i].y;
				if (points[i].y > ymax) ymax = points[i].y;
			}
			float dx = xmax - xmin;
			float dy = ymax - ymin;
			float maxD = (dx > dy) ? dx : dy;
			cVec2 mid{ (xmin + xmax) * 0.5f, (ymin + ymax) * 0.5f };

			// Create a large super triangle
			cVec2 p1 = { mid.x - 2 * maxD, mid.y - maxD };
			cVec2 p2 = { mid.x, mid.y + 2 * maxD };
			cVec2 p3 = { mid.x + 2 * maxD, mid.y - maxD };
			const cDelTriangle st = { p1, p2, p3 };
			triangles = { st }; // Initialize triangles while adding bounding triangle
			// Triangulate each vertex
			for (unsigned i = 0; i < inCount; ++i)
			{
				insertPoint(points[i]);
			}
			// Remove triangles that share edges with super triangle
			// (this includes the super triangle itself)
			std::vector<cDelTriangle> filteredTris{};
			for (int i = 0; i < triangles.size(); ++i)
			{
				const cDelTriangle& tri = triangles[i];
				// remove any triangles that share a vertex with the super triangle
				if (
					tri.a == st.a || tri.a == st.b || tri.a == st.c ||
					tri.b == st.a || tri.b == st.b || tri.b == st.c ||
					tri.c == st.a || tri.c == st.b || tri.c == st.c
					)
					continue;

				filteredTris.push_back(tri);
			}
			triangles.swap(filteredTris);
		}

		void insertPoint(const cVec2& inPoint)
		{
			std::vector<cDelEdge> edges{};
	
			std::vector<cDelTriangle> bad_tris{};
			for (int i = 0; i < triangles.size(); ++i)
			{
				const cDelTriangle& tri = triangles[i];
				if (tri.circumcircleContains(inPoint))
				{
					edges.emplace_back(tri.a, tri.b);
					edges.emplace_back(tri.b, tri.c);
					edges.emplace_back(tri.c, tri.a);
					bad_tris.push_back(tri);
				}
			}
			
			std::vector<cDelTriangle> n_tris;
			n_tris.reserve(triangles.size() - bad_tris.size());
			for (const auto& tri : triangles) {
				if (std::find(bad_tris.begin(), bad_tris.end(), tri) == bad_tris.end()) {
					n_tris.push_back(tri);
				}
			}
			
			// Get unique edges, removing duplicates
			std::sort(edges.begin(), edges.end());
			std::vector<cDelEdge> uniqueEdges;
			for (size_t i = 0; i < edges.size(); ++i) {
				if (i + 1 < edges.size() && edges[i] ==
					edges[i + 1])
				{
					++i;
				}
				else {
					uniqueEdges.push_back(edges[i]);
				}
			}
			edges.swap(uniqueEdges);

			// Create new triangles from the unique edges and new vertex
			for (int i = 0; i < edges.size(); ++i)
			{
				const cDelEdge& e = edges[i];
				n_tris.emplace_back(e.p1, e.p2, inPoint);
			}
			// replace the old list of triangles with the new one
			triangles = n_tris;
		}
	};

}

