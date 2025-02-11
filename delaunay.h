#pragma once
#define NOMINMAX
#include "chioriMath.h"
#include "chioriAllocator.h"
#include "delaunator.hpp"
#include <vector>


// TODO: Optimize and convert to using allocators

namespace chiori
{
	struct cDelTriangle
	{
		cDelTriangle(const cVec2& inA = cVec2::zero, const cVec2& inB = cVec2::zero, const cVec2& inC = cVec2::zero) :
			a{ inA }, b{ inB }, c{ inC }
		{
			if (!isCounterClockwise())
				std::swap(b, c);
		}
		bool isCounterClockwise() const { return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y) > 0; }
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
			p1{ inP1 }, p2{ inP2 }
		{
			if (p1.x > p2.x || (p1.x == p2.x && p1.y > p2.y))
				std::swap(p1, p2);
		}

		bool operator==(const cDelEdge& other) const {
			return (p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1);
		}

		cVec2 p1, p2;
	};

	struct EdgeHash {
		size_t operator()(const cDelEdge& edge) const {
			// Ensure order is consistent (unordered pair)
			size_t h1 = std::hash<float>()(edge.p1.x) ^ (std::hash<float>()(edge.p1.y) << 1);
			size_t h2 = std::hash<float>()(edge.p2.x) ^ (std::hash<float>()(edge.p2.y) << 1);

			// Combine hashes ensuring the same hash for (p1, p2) and (p2, p1)
			return h1 ^ h2;
		}
	};

	class cDelaunayTriangulation
	{
		cDelTriangle getSuperTri(const cVec2* points, unsigned inCount)
		{
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

			return { p1,p2,p3 };
		}

		cDelTriangle growSuperTriangle(const cDelTriangle& tri, const cVec2& pt)
		{
			float xmin = c_min(tri.a.x, c_min(tri.b.x, c_min(tri.c.x, pt.x)));
			float ymin = c_min(tri.a.y, c_min(tri.b.y, c_min(tri.c.y, pt.y)));
			float xmax = c_max(tri.a.x, c_max(tri.b.x, c_max(tri.c.x, pt.x)));
			float ymax = c_max(tri.a.y, c_max(tri.b.y, c_max(tri.c.y, pt.y)));

			float dx = xmax - xmin;
			float dy = ymax - ymin;
			float maxD = (dx > dy) ? dx : dy;
			cVec2 mid{ (xmin + xmax) * 0.5f, (ymin + ymax) * 0.5f };

			// Create a larger super triangle
			cVec2 p1 = { mid.x - 2 * maxD, mid.y - maxD };
			cVec2 p2 = { mid.x, mid.y + 2 * maxD };
			cVec2 p3 = { mid.x + 2 * maxD, mid.y - maxD };

			return { p1,p2,p3 };
		}

		void filterTriangles(cDelTriangle inFilteredTri)
		{
			std::vector<cDelTriangle> filteredTris{};
			for (int i = 0; i < triangles.size(); ++i)
			{
				const cDelTriangle& tri = triangles[i];
				// remove any triangles that share a vertex with the input filtered triangle (usually the super triangle)
				if (
					tri.a == inFilteredTri.a || tri.a == inFilteredTri.b || tri.a == inFilteredTri.c ||
					tri.b == inFilteredTri.a || tri.b == inFilteredTri.b || tri.b == inFilteredTri.c ||
					tri.c == inFilteredTri.a || tri.c == inFilteredTri.b || tri.c == inFilteredTri.c
					)
					continue;

				filteredTris.push_back(tri);
			}
			triangles = filteredTris;
		}

		void filterEdges(std::vector<cDelEdge>& edges)
		{
			std::vector<cDelEdge> uniqueEdges{};
			for (int i = 0; i < edges.size(); ++i)
			{
				bool isUnique = true;
				for (int j = 0; j < edges.size(); ++j)
				{
					if (i == j)
						continue;

					if (edges[i] == edges[j])
					{
						isUnique = false;
						break;
					}
				}
				if (isUnique)
				{
					uniqueEdges.push_back(edges[i]);
				}
			}
			edges = uniqueEdges;
		}


	public:
		std::vector<cDelTriangle> triangles{};
		std::vector<cVec2> vertices{};

		void triangulate(const cVec2* points, unsigned inCount)
		{
			vertices.assign(points, points + inCount);
			// check for valid input
			cassert(points);
			cassert(inCount > 0);
			// Create bounding 'super' triangle
			const cDelTriangle st = getSuperTri(points, inCount);
			triangles = { st }; // Initialize triangles while adding bounding triangle
			// Triangulate each vertex
			for (unsigned i = 0; i < inCount; ++i)
			{
				insertPoint(points[i]);
			}
			// Remove triangles that share edges with super triangle
			// (this auto includes the super triangle itself)
			filterTriangles(st);
		}

		void insertPoint(const cVec2& inPoint)
		{
			std::vector<cDelEdge> edges{};
			std::vector<cDelTriangle> n_tris{};

			// Remove triangles with circumcircles containing the vertex
			bool withinBounds = false;
			for (int i = 0; i < triangles.size(); ++i)
			{
				const cDelTriangle& tri = triangles[i];
				if (tri.circumcircleContains(inPoint))
				{
					edges.emplace_back(tri.a, tri.b);
					edges.emplace_back(tri.b, tri.c);
					edges.emplace_back(tri.c, tri.a);
					withinBounds = true; // the point was found within the circumcircle of a triangle in the list
				}
				else
					n_tris.push_back(tri);
			}

			// Get unique edges, removing duplicates
			filterEdges(edges);
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


	std::vector<std::vector<cVec2>> triangulateDelaunator(const std::vector<cVec2>& points)
	{
		if (points.size() < 3)
			return {};

		std::vector<float> coords;
		std::vector<std::vector<cVec2>> triangles;

		// Convert cVec2 points into flat double array
		for (const auto& p : points) {
			coords.push_back(p.x);
			coords.push_back(p.y);
		}

		// Perform Delaunay triangulation
		delaunator::Delaunator d(coords);

		// Convert output triangles back into cVec2
		for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
			cVec2 v0(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1]);
			cVec2 v1(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]);
			cVec2 v2(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]);

			triangles.push_back({ v0, v1, v2 });
		}

		return triangles;
	}
}

