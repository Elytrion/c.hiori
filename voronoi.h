#pragma once
#include "delaunay.h"

namespace chiori
{
	struct cVEdge
	{
		cVec2 origin; // origin of the edge
		cVec2 endDir; // if infinite, endDir is the direction of the edge, otherwise it is the end point
		bool infinite{ false }; // if true, the edge is infinite
		int nextEdge{ -1 }; // index of the next edge in the edge list
	};

	struct cVCell
	{
		cVec2 site;				// the centriod/site of the cell
		int edgeList{ -1 };	// the index of the latest edge in the cells edge LL
	};

	class cVoronoiDiagram
	{
	public:
		std::vector<cVCell> cells; // Each cell corresponds to a Voronoi region
		std::vector<cVEdge> edges; // All edges in the diagram
		cDelaunayTriangulation delaunay; // Delaunay triangulation

		cVoronoiDiagram(const std::vector<cVec2>& points) : delaunay{ points } {
			initialize(points);
		}

		void insertPoint(const cVec2& p)
		{
			delaunay.insertPoint(p);
			reconstructVoronoi();
		}
		void removePoint(const cVec2& p)
		{
			delaunay.removePoint(p);
			reconstructVoronoi();
		}

	private:
		void initialize(const std::vector<cVec2>& points)
		{
			if (points.empty()) return;

			// Special case handling
			if (points.size() == 1) {
				handleSinglePoint(points[0]);
				return;
			}
			if (points.size() == 2) {
				handleTwoPoints(points[0], points[1]);
				return;
			}

			// Use Delaunay triangulation for >2 points
			delaunay = cDelaunayTriangulation(points);
			reconstructVoronoi();
		}

		// Edge case 1: Single point (no edges)
		void handleSinglePoint(const cVec2& p)
		{
			cells.push_back({ p, -1 });
		}
		// Edge case 2: Two points (one infinite edge)
		void handleTwoPoints(const cVec2& p1, const cVec2& p2)
		{
			cells.push_back({ p1, 0 });
			cells.push_back({ p2, 1 });

			cVec2 midPoint = (p1 + p2) * 0.5;
			cVec2 direction = { p2.y - p1.y, -(p2.x - p1.x) }; // Perpendicular

			edges.push_back({ midPoint, direction, true, -1 });
			edges[0].nextEdge = 1;
			edges.push_back({ midPoint, -direction, true, -1 });
		}

		struct HashPair {
			size_t operator()(const std::pair<cVec2, cVec2>& p) const {
				// Hashing two floating-point numbers
				size_t h1 = std::hash<float>()(p.first.x) ^ (std::hash<float>()(p.first.y) << 1);
				size_t h2 = std::hash<float>()(p.second.x) ^ (std::hash<float>()(p.second.y) << 1);
				return h1 ^ (h2 << 1); // Combine hashes
			}
		};
		void reconstructVoronoi()
		{
			cells.clear();
			edges.clear();
			std::unordered_map<std::pair<cVec2, cVec2>, int, HashPair> edgeMap;

			// Iterate through Delaunay triangles
			for (const auto& tri : delaunay.triangulation) {
				// Compute the circumcenter (Voronoi centroid)
				cVec2 centroid = tri.circumcenter();

				// Create a new Voronoi cell
				int cellIdx = cells.size();
				cells.push_back({ centroid, -1 });

				// Process each triangle edge
				processVoronoiEdge(tri.a, tri.b, centroid, cellIdx, edgeMap);
				processVoronoiEdge(tri.b, tri.c, centroid, cellIdx, edgeMap);
				processVoronoiEdge(tri.c, tri.a, centroid, cellIdx, edgeMap);
			}
		}

		bool isEdgeSharedByAnotherTriangle(const cVec2& p1, const cVec2& p2) {
			int sharedCount = 0;
			for (const auto& tri : delaunay.triangulation) {
				if ((tri.a == p1 && tri.b == p2) || (tri.b == p1 && tri.a == p2) ||
					(tri.b == p1 && tri.c == p2) || (tri.c == p1 && tri.b == p2) ||
					(tri.c == p1 && tri.a == p2) || (tri.a == p1 && tri.c == p2)) {
					sharedCount++;
				}
			}
			return sharedCount > 1; // More than one triangle shares this edge
		}

		// Process Voronoi edges based on Delaunay edges
		void processVoronoiEdge(const cVec2& p1, const cVec2& p2, const cVec2& centroid, int cellIdx,
			std::unordered_map<std::pair<cVec2, cVec2>, int, HashPair>& edgeMap)
		{

			// Ensure consistent ordering of edges
			auto orderedEdge = [](const cVec2& a, const cVec2& b) -> std::pair<cVec2, cVec2> {
				return (a.x < b.x || (a.x == b.x && a.y < b.y)) ? std::make_pair(a, b) : std::make_pair(b, a);
				};

			auto key = orderedEdge(p1, p2);

			if (edgeMap.find(key) == edgeMap.end()) {
				// New edge - store its index in edgeMap
				int edgeIdx = edges.size();
				edges.push_back({ centroid, {}, false, -1 }); // Temporary end point

				// Attach edge to the cell’s linked list
				edges[edgeIdx].nextEdge = cells[cellIdx].edgeList;
				cells[cellIdx].edgeList = edgeIdx;

				// Store in edgeMap for lookup
				edgeMap[key] = edgeIdx;
			}
			else {
				// Update existing edge with correct second point
				int edgeIdx = edgeMap[key];
				edges[edgeIdx].endDir = centroid;

				// If no second triangle shares this edge, it's a boundary edge
				if (!isEdgeSharedByAnotherTriangle(p1, p2)) {
					edges[edgeIdx].infinite = true;
					cVec2 edgeLine = p2 - p1;
					cVec2 edgeNormal = { edgeLine.y, -edgeLine.x };
					edges[edgeIdx].endDir = edgeNormal;
				}
			}

		}
	};
}