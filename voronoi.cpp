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
			std::vector<unsigned> cellVerticesIndices;
			bool isInfinite = (hullSet.count(seedIndex) > 0); // a cell is infinite if is the at the edge of the convex hull of the point cloud

			unsigned verticesCount = vertices.size();
			for (int i = 0; i < verticesCount; ++i)
			{
				const cVVert& vertex = vertices[i];
				if (std::find(vertex.seedIndices.begin(), vertex.seedIndices.end(), seedIndex) != vertex.seedIndices.end()) {
					cellVerticesIndices.push_back(i);
				}
			}

			// cleanup into CCW convex + infinite cell setting
			if (!cellVerticesIndices.empty()) {
				cVec2 seed = v_points[seedIndex];

				
				std::sort(cellVerticesIndices.begin(), cellVerticesIndices.end(), [&](unsigned& ia, unsigned& ib)
					{
						auto& a = vertices[ia]; auto& b = vertices[ib];
						return atan2(a.site.y - seed.y, a.site.x - seed.x) < atan2(b.site.y - seed.y, b.site.x - seed.x);
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

					int vertCount = cellVerticesIndices.size();
					int infVIndexA = -1, infVIndexB = -1;
					int infEIndexA = -1, infEIndexB = -1;
					cVCell& cell = cells[seedIndex];
					cell = { seedIndex, cellVerticesIndices, isInfinite };				
					if (vertCount == 1)
						continue; // degenerate cell, handle externally!

					if (vertCount == 2) // straight line, unique case we can handle quickly
					{
						const cVVert& v1 = vertices[cellVerticesIndices[0]];
						const cVVert& v2 = vertices[cellVerticesIndices[1]];
						infVIndexA = 0; infVIndexB = 1;
						// get the outer facing line normal, the one in the same dir as the seed
						const cVec2 line = v2.site - v1.site;
						cVec2 tangent = line.tangent().normalized();
						if (tangent.dot(v_points[seedIndex] - v1.site) < 0)
							tangent = -tangent;
						
						float bestDotA = -FLT_MAX, bestDotB = -FLT_MAX;
						for (unsigned edgeIdx : v1.edgeIndices) // go through its 3 edges
						{
							if (edges[edgeIdx].infinite) // find only the infinite edges
							{
								cVec2 edgeDir = edges[edgeIdx].endDir.normalized(); // get the dir of the edge
								float dot = (edgeDir.dot(tangent));
								if (dot > bestDotA)
								{
									bestDotA = dot;
									infEIndexA = edgeIdx;
								}
							}
						}

						for (unsigned edgeIdx : v2.edgeIndices) // go through its 3 edges
						{
							if (edges[edgeIdx].infinite) // find only the infinite edges
							{
								cVec2 edgeDir = edges[edgeIdx].endDir.normalized(); // get the dir of the edge
								float dot = (edgeDir.dot(tangent));
								if (dot > bestDotB)
								{
									bestDotB = dot;
									infEIndexB = edgeIdx;
								}
							}
						}

						cell.infVertA = infVIndexA; // Vertex A index
						cell.infEdgeA = infEIndexA; // Edge A index
						cell.infVertB = infVIndexB; // Vertex B index
						cell.infEdgeB = infEIndexB; // Edge B index
						continue;
					}
								
					for (int i = 0; i < vertCount; ++i)
					{
						const cVVert& v = vertices[cellVerticesIndices[i]];
						if (isInfiniteVert(v)) // if the vertex has infinite edges
						{
							for (unsigned edgeIdx : v.edgeIndices) // go through its 3 edges
							{
								if (edges[edgeIdx].infinite) // find only the infinite edges
								{
									cVec2 edgeDir = edges[edgeIdx].endDir.normalized(); // get the dir of the edge

									int nextIndex = (i + 1) % vertCount;
									int prevIndex = (i - 1 + vertCount) % vertCount;
									cVec2 nxtVert = vertices[cellVerticesIndices[nextIndex]].site;
									cVec2 prevVert = vertices[cellVerticesIndices[prevIndex]].site;
									cVec2 edgeToNext = (nxtVert - v.site).normalized();
									cVec2 edgeFromPrev = (v.site - prevVert).normalized();
									bool ccwNext = edgeDir.dot(-edgeToNext.tangent()) >= 0;
									bool ccwPrev = edgeDir.dot(-edgeFromPrev.tangent()) >= 0;
									if (ccwNext || ccwPrev) // if it doesnt keep CCW rotation for both the next or prev edge, it cannot be the correct edge
									{
										if (infVIndexA < 0)
										{
											infVIndexA = i;
											infEIndexA = edgeIdx;
										}
										else
										{
											infVIndexB = i;
											infEIndexB = edgeIdx;
										}
									}
								}
							}
						}
					}

					cell.infVertA = infVIndexA; // Vertex A index
					cell.infEdgeA = infEIndexA; // Edge A index
					cell.infVertB = infVIndexB; // Vertex B index
					cell.infEdgeB = infEIndexB; // Edge B index
					
					//infIndexA = infiniteEdges[0].second;
					//infIndexB = infiniteEdges[1].second;
					////// Step 1: Find the first infinite vertex
					////for (int i = 0; i < vertCount; ++i)
					////{
					////	if (isInfiniteVert(cellVertices[i]))
					////	{
					////		infIndexA = i;
					////		break;
					////	}
					////}
					////// Step 2: Find the second infinite vertex (always adjacent)
					////infIndexB = (infIndexA + 1) % vertCount;	
					////// Edge case: The last vertex might be the second infinite one
					////if (infIndexA == 0 && isInfiniteVert(cellVertices[vertCount - 1]))
					////{
					////	infIndexB = 0;
					////	infIndexA = vertCount - 1;
					////}

					//// Step 3: Reorder vertices such that the first two are always infinite

					//std::vector<cVVert> rearranged;
					//rearranged.reserve(vertCount);
					//rearranged.push_back(cellVerticesIndices[infIndexA]);
					//rearranged.push_back(cellVerticesIndices[infIndexB]);
					//std::cout << "Indices A: " << infIndexA << "/" << vertCount - 1 << std::endl;
					//std::cout << "Indices B: " << infIndexB << "/" << vertCount - 1<< std::endl;
					//// Step 4: Add remaining vertices in CCW order
					//int index = (infIndexB + 1) % vertCount;
					//while (index != infIndexA)
					//{
					//	rearranged.push_back(cellVerticesIndices[index]);
					//	index = (index + 1) % vertCount;
					//}
					//std::cout << vertCount << "/" << rearranged.size() << std::endl;
					//// Step 5: Swap rearranged list into `cellVertices`
					//cellVerticesIndices.swap(rearranged);
				}
				else
					cells[seedIndex] = { seedIndex, cellVerticesIndices, isInfinite };
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
