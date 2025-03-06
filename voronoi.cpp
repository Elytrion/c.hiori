#include "pch.h"
#include "voronoi.h"
#include "aabb.h"
#include <numeric>

namespace chiori
{
	float Orientation(cVec2 a, cVec2 b, cVec2 c) {
		return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
	}
	// Compute convex hull using Graham's Scan
	std::vector<unsigned> ComputeConvexHull(const std::vector<cVec2>& points) {
		std::vector<unsigned> hull;

		if (points.size() < 3) return hull; // Convex hull is undefined for less than 3 points

		// Find the lowest point (leftmost in case of tie)
		unsigned start = 0;
		for (unsigned i = 1; i < points.size(); i++) {
			if (points[i].y < points[start].y || (points[i].y == points[start].y && points[i].x < points[start].x)) {
				start = i;
			}
		}

		// Sort points based on polar angle relative to start point
		std::vector<unsigned> sorted(points.size());
		std::iota(sorted.begin(), sorted.end(), 0);
		std::sort(sorted.begin(), sorted.end(), [&](unsigned a, unsigned b) {
			float cross = Orientation(points[start], points[a], points[b]);
			return (cross > 0) || (cross == 0 && (points[a] - points[start]).sqrMagnitude() < (points[b] - points[start]).sqrMagnitude());
			});

		// Build the hull
		for (unsigned index : sorted) {
			while (hull.size() >= 2 && Orientation(points[hull[hull.size() - 2]], points[hull.back()], points[index]) <= 0) {
				hull.pop_back();
			}
			hull.push_back(index);
		}

		return hull; // Returns the indices of points in the convex hull
	}

	// omg this an expensive algorithm, lets only use it if absolutely required
	std::vector<cVCell> cVoronoiDiagram::getCells() const
	{
		unsigned seedCount = v_points.size();
		std::vector<cVCell> cells(seedCount);

		std::vector<unsigned> convexHull = ComputeConvexHull(v_points);
		std::unordered_set<unsigned> hullSet(convexHull.begin(), convexHull.end()); // Fast lookup

		for (unsigned seedIndex = 0; seedIndex < seedCount; ++seedIndex) {
			std::vector<cVVert> cellVertices;
			bool isInfinite = (hullSet.count(seedIndex) > 0);;

			for (const cVVert& vertex : vertices) {
				if (std::find(vertex.seedIndices.begin(), vertex.seedIndices.end(), seedIndex) != vertex.seedIndices.end()) {
					cellVertices.push_back(vertex);
				}
			}

			// cleanup into CCW convex + infinite cell setting
			if (!cellVertices.empty()) {
				cVec2 centroid = v_points[seedIndex];

				std::sort(cellVertices.begin(), cellVertices.end(), [centroid](const cVVert& a, const cVVert& b)
					{
						return atan2(a.site.y - centroid.y, a.site.x - centroid.x) < atan2(b.site.y - centroid.y, b.site.x - centroid.x);
					});

				if (isInfinite) 
				{
					auto isInfiniteVert = [&](const cVVert& vert) -> bool
						{
							for (unsigned i : vert.edgeIndices)
							{
								if (edges[i].infinite)
									return true;
							}
							return false;
						};

					int vertCount = cellVertices.size();
					int infIndexA = -1, infIndexB = -1;

					if (vertCount == 1)
					{
						cells[seedIndex] = { seedIndex, cellVertices, isInfinite };
						continue; // degenerate cell, handle externally!
					}

					std::vector<std::pair<float, int>> infiniteEdges; // (dot product, vertex index)

					for (int i = 0; i < vertCount; ++i)
					{
						if (isInfiniteVert(cellVertices[i]))
						{
							cVec2 toSeed = (v_points[seedIndex] - cellVertices[i].site).normalized();

							for (unsigned edgeIdx : cellVertices[i].edgeIndices)
							{
								if (edges[edgeIdx].infinite)
								{
									cVec2 edgeDir = edges[edgeIdx].endDir.normalized();
									float dot = edgeDir.dot(toSeed); // Find best-aligned infinite edge
									infiniteEdges.emplace_back(dot, i);
								}
							}
						}
					}

					std::sort(infiniteEdges.begin(), infiniteEdges.end(),
						[](const std::pair<float, int>& a, const std::pair<float, int>& b) {
							return a.first > b.first; // Higher dot product = better alignment
						});

					infIndexA = infiniteEdges[0].second;
					infIndexB = infiniteEdges[1].second;

					//// Step 1: Find the first infinite vertex
					//for (int i = 0; i < vertCount; ++i)
					//{
					//	if (isInfiniteVert(cellVertices[i]))
					//	{
					//		infIndexA = i;
					//		break;
					//	}
					//}
					//// Step 2: Find the second infinite vertex (always adjacent)
					//infIndexB = (infIndexA + 1) % vertCount;	
					//// Edge case: The last vertex might be the second infinite one
					//if (infIndexA == 0 && isInfiniteVert(cellVertices[vertCount - 1]))
					//{
					//	infIndexB = 0;
					//	infIndexA = vertCount - 1;
					//}

					// Step 3: Reorder vertices such that the first two are always infinite

					std::vector<cVVert> rearranged;
					rearranged.reserve(vertCount);
					rearranged.push_back(cellVertices[infIndexA]);
					rearranged.push_back(cellVertices[infIndexB]);
					std::cout << "Indices A: " << infIndexA << "/" << vertCount - 1 << std::endl;
					std::cout << "Indices B: " << infIndexB << "/" << vertCount - 1<< std::endl;
					// Step 4: Add remaining vertices in CCW order
					int index = (infIndexB + 1) % vertCount;
					while (index != infIndexA)
					{
						rearranged.push_back(cellVertices[index]);
						index = (index + 1) % vertCount;
					}
					std::cout << vertCount << "/" << rearranged.size() << std::endl;
					// Step 5: Swap rearranged list into `cellVertices`
					cellVertices.swap(rearranged);
				}

				cells[seedIndex] = { seedIndex, cellVertices, isInfinite };
			}
		}

		return cells;
	}
	
	// check if a point is on the LEFT side of an edge
	bool is_inside(cVec2 point, cVec2 a, cVec2 b) {
		return (cross(a - b, point) + cross(b, a)) < 0.0f;
	}
	// calculate intersection point
	cVec2 intersection(cVec2 a1, cVec2 a2, cVec2 b1, cVec2 b2) {
		return ((b1 - b2) * cross(a1, a2) - (a1 - a2) * cross(b1, b2)) *
			(1.0f / cross(a1 - a2, b1 - b2));
	}
	// Sutherland-Hodgman clipping (generic for all polygons, but primarily used for just lines here)
	// TODO: optimise to remove std::vector is place for something else?
	std::vector<cVec2> suther_land_hodgman(
		std::vector<cVec2> subject_polygon, std::vector<cVec2> clip_polygon) {
		if (clip_polygon.empty() || subject_polygon.empty()) {
			return {};
		}

		std::vector<cVec2> ring{ subject_polygon.begin(), subject_polygon.end() };

		cVec2 p1 = clip_polygon[clip_polygon.size() - 1];

		std::vector<cVec2> input;

		for (cVec2 p2 : clip_polygon) {
			input.clear();
			input.insert(input.end(), ring.begin(), ring.end());
			if (input.size() < 1)
				return {};
			cVec2 s = input[input.size() - 1];

			ring.clear();

			for (cVec2 e : input) {
				if (is_inside(e, p1, p2)) {
					if (!is_inside(s, p1, p2)) {
						ring.push_back(intersection(p1, p2, s, e));
					}

					ring.push_back(e);
				}
				else if (is_inside(s, p1, p2)) {
					ring.push_back(intersection(p1, p2, s, e));
				}

				s = e;
			}

			p1 = p2;
		}

		return ring;
	}

	std::vector<cVec2> ClipVoronoiWithPolygon(const cVoronoiDiagram& inPattern, const cVec2* p_vertices, const cVec2* p_normals, int p_count)
	{
		cTransform ixf; // identity
		cAABB bounds = CreateAABBHull(p_vertices, p_count, ixf);
		float extensionFactor = bounds.getExtents().sqrMagnitude() + 1; // we add 1 to avoid any `on the edge` issues for bounds with a sqrmag of 1 or 0
		std::unordered_set<cVec2, cVec2Hash> verts;
		for (const auto& edge : inPattern.edges)
		{
			cVec2 o = edge.origin;
			cVec2 e = (edge.infinite) ? (o + edge.endDir * extensionFactor) : edge.endDir;

			std::vector<cVec2> poly{ p_vertices, p_vertices + p_count };
			std::vector<cVec2> ce = suther_land_hodgman({ o,e }, poly);
			for (auto& p : ce)
			{
				verts.insert(p);
			}
		}
		return { verts.begin(), verts.end() };
	}
	

}
