#pragma once
#include "chioriMath.h"
#include <vector>
#include <unordered_map>

namespace chiori
{
	struct cDelTriangle
	{
		cVec2 a, b, c;
		cVec2 circumcenter() const
		{
			float dA = a.dot(a);
			float dB = b.dot(b);
			float dC = c.dot(c);
			float det = 2.0f * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
			return { (dA * (b.y - c.y) + dB * (c.y - a.y) + dC * (a.y - b.y)) / det,
								 (dA * (c.x - b.x) + dB * (a.x - c.x) + dC * (b.x - a.x)) / det };
		}
		bool isPointInCircumcircle(const cVec2& p) const
		{
			const cVec2 center = circumcenter();
			float radiusSqr = distanceSqr(a, center);
			float distSqr = distanceSqr(p, center);
			return distSqr <= radiusSqr;
		}
		bool operator==(const cDelTriangle& other) const {
			return (a == other.a && b == other.b && c == other.c) ||
				(a == other.b && b == other.c && c == other.a) || // Handle cyclic order
				(a == other.c && b == other.a && c == other.b);
		}
	};

	struct cDelEdge
	{
		cVec2 p1, p2;

		bool operator==(const cDelEdge& other) const {
			return (p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1);
		}
	};

}
namespace std {
	template <>
	struct hash<chiori::cDelEdge> {
		size_t operator()(const chiori::cDelEdge& e) const {
			size_t h1 = hash<float>()(e.p1.x) ^ (hash<float>()(e.p1.y) << 1);
			size_t h2 = hash<float>()(e.p2.x) ^ (hash<float>()(e.p2.y) << 1);
			return h1 ^ h2; // Ensures (A, B) == (B, A)
		}
	};
}
namespace chiori
{
	class cDelaunayTriangulation
	{
	public:
		std::vector<cDelTriangle> triangulation {};

		cDelaunayTriangulation(const std::vector<cVec2>& points = {}) {
			if (!points.empty())
				init(points);
		}

		void init(const std::vector<cVec2>& points)
		{
			cassert(points.size() >= 3);
			initializeTriangulation(points);
		}

		void insertPoint(const cVec2& p)
		{
			cassert(triangulation.size() > 0);

			std::vector<cDelTriangle> badTriangles;
			std::unordered_map<cDelEdge, int> edgeCount;

			// Step 1: Find bad triangles whose circumcircle contains p
			for (const cDelTriangle& t : triangulation) {
				if (t.isPointInCircumcircle(p)) {
					badTriangles.push_back(t);
					trackEdges(edgeCount, t);
				}
			}

			// Step 2: Remove bad triangles from triangulation
			removeTriangles(badTriangles);

			// Step 3: Form new triangles using the polygonal cavity boundary
			addNewTriangles(p, edgeCount);
		}

		void removePoint(const cVec2& p)
		{
			cassert(triangulation.size() > 0);

			std::vector<cDelTriangle> badTriangles;
			std::unordered_map<cDelEdge, int> edgeCount;

			// Step 1: Find all triangles containing the point
			for (const cDelTriangle& t : triangulation) {
				if (t.a == p || t.b == p || t.c == p) {
					badTriangles.push_back(t);
					trackEdges(edgeCount, t);
				}
			}

			// Step 2: Remove affected triangles
			removeTriangles(badTriangles);

			// Step 3: Re-triangulate the hole
			for (const auto& edge : edgeCount) {
				if (edge.second == 1) { // Boundary edge
					cVec2 mid = { (edge.first.p1.x + edge.first.p2.x) / 2,
								  (edge.first.p1.y + edge.first.p2.y) / 2 };
					addNewTriangles(mid, edgeCount);
				}
			}
		}

	private:
		float sTriFactor = 100.0f; // how much to grow the super triangle bounding box

		// Initialize triangulation using a super-triangle
		void initializeTriangulation(const std::vector<cVec2>& points) {
			// Step 1: Create a super-triangle that encompasses all points
			float minX = points[0].x, maxX = points[0].x;
			float minY = points[0].y, maxY = points[0].y;
			for (const auto& p : points) {
				minX = c_min(minX, p.x);
				maxX = c_max(maxX, p.x);
				minY = c_min(minY, p.y);
				maxY = c_max(maxY, p.y);
			}

			cVec2 p1 = { minX - sTriFactor, minY - sTriFactor };
			cVec2 p2 = { maxX + sTriFactor, minY - sTriFactor };
			cVec2 p3 = { (minX + maxX) / 2, maxY + sTriFactor };

			cDelTriangle superTriangle = { p1, p2, p3 };
			triangulation.push_back(superTriangle);

			// Step 2: Insert all points dynamically
			for (const auto& p : points) {
				insertPoint(p);
			}

			// Step 3: Remove super-triangle-related edges
			triangulation.erase(std::remove_if(triangulation.begin(), triangulation.end(),
				[&](const cDelTriangle& t) {
					return (t.a == superTriangle.a || t.b == superTriangle.a || t.c == superTriangle.a);
				}), triangulation.end());
		}
		// Track edges of a triangle
		void trackEdges(std::unordered_map<cDelEdge, int>& edgeCount, const cDelTriangle& t)
		{
			edgeCount[{t.a, t.b}]++;
			edgeCount[{t.b, t.c}]++;
			edgeCount[{t.c, t.a}]++;
		}
		// Remove bad triangles
		void removeTriangles(const std::vector<cDelTriangle>&badTriangles)
		{
			for (const cDelTriangle& t : badTriangles) {
				auto it = std::find(triangulation.begin(), triangulation.end(), t);
				if (it != triangulation.end()) {
					triangulation.erase(it);
				}
			}
		}
		// Add new triangles
		void addNewTriangles(const cVec2& p, const std::unordered_map<cDelEdge, int>& edgeCount)
		{
			for (const auto& edge : edgeCount) {
				if (edge.second == 1) { // Unique boundary edges
					cDelTriangle newTriangle = { edge.first.p1, edge.first.p2, p };
					triangulation.push_back(newTriangle);
				}
			}
		}
	};

}

