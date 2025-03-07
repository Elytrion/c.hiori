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
				const cVec2& seed = v_points[seedIndex];		
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

					// CASE 1: Degenerate cell, 1 vertex, 2 infinite edges that comprise the entire cell
					if (vertCount == 1)
					{
						// get infinite edges
						for (unsigned edgeIdx : vertices[cellVerticesIndices[0]].edgeIndices)
						{
							if (edges[edgeIdx].infinite)
							{
								if (infVIndexA < 0)
									infEIndexA = edgeIdx;
								else
									infEIndexB = edgeIdx;
							}
						}
						cell.infEdgeA = infEIndexA; // Edge A index
						cell.infEdgeB = infEIndexB; // Edge B index
						continue;
					}

					// CASE 2: Straight line cell, unique case we can handle quickly,
					// since the edge vertices must contain the infinite edges that seal the cell
					// simply looking for the correct edges that are in closest direction as the seed point
					if (vertCount == 2)
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
								
					// CASE 3: General case, 3 or more vertices. Much more complex, we need to find the correct end vertices with the correct infinite edges
					// If a vertice has infinite edges in a cell with > 2 vertices, its gives us the following subcases
					// 1. 2 infinite edges, 1 finite edge
					//		This means the finite edge HAS to make up the edge of the cell
					//		(else it means the cell has 2 infinite edges in 1 vertex, which is a degenerate vertex, filtered out earlier)
					//		meaning one of the two infinite edges is the actual edge of the cell, the other points away from the cell.
					//		If we take the inward facing tangent of the finite edge (pointing to the seed point),
					//		the infinite edge that has the highest dot product with the tangent is the edge of the cell
					// 2. 2 finite edges, 1 infinite edge
					//	    This can be further broken down into 2 problems:
					//		a. 2 finite edges make up the border of the cell, the infinite edge points away from the cell,
					//			and hence is irrelevant, we need to ensure we dont store this as a valid infinite edge
					//		b. 1 finite edge and the infinite edge make up the border of the cell, and the other finite edge is the border of another cell,
					//		   thus we must make sure this infinite edge is stored as valid
					// 
					//      In the case of the valid infinite edge, the ccw check will at least return 1 true (Preservation of convexity).
					//		Else, if its the infinite edge pointing away, the ccw check will return completely false!
					// If we solve these subcases, it should solve the entire infinite cell problem

					for (int i = 0; i < vertCount; ++i)
					{
						const cVVert& v = vertices[cellVerticesIndices[i]];
						if (isInfiniteVert(v)) // if the vertex has infinite edges
						{
							// cases: 1 infinite edge vs 2 infinite edges (Cannot have 3 infinite edges, nor 3 finite edges here (An infinite cell that is not degenerate)
							int infEdge1 = -1, infEdge2 = -1;
							for (unsigned edgeIdx : v.edgeIndices) // go through its 3 edges
							{
								if (edges[edgeIdx].infinite) // find only the infinite edges
								{
									if (infEdge1 < 0)
										infEdge1 = edgeIdx;
									else
										infEdge2 = edgeIdx;
								}
							}

							if (infEdge2 < 0) // subcase 2 single infinite edge case
							{
								//	    This can be further broken down into 2 problems:
								//		a. 2 finite edges make up the border of the cell, the infinite edge points away from the cell,
								//			and hence is irrelevant, we need to ensure we dont store this as a valid infinite edge
								//		b. 1 finite edge and the infinite edge make up the border of the cell, and the other finite edge is the border of another cell,
								//		   thus we must make sure this infinite edge is stored as valid
								//      In the case of the valid infinite edge, the ccw check will at least return 1 true (Preservation of convexity).
								//		Else, if its the infinite edge pointing away, the ccw check will return completely false!

								cVec2 edgeDir = edges[infEdge1].endDir.normalized(); // get the dir of the infinite edge

								int nextIndex = (i + 1) % vertCount;
								int prevIndex = (i - 1 + vertCount) % vertCount;
								cVec2 nxtVert = vertices[cellVerticesIndices[nextIndex]].site;
								cVec2 prevVert = vertices[cellVerticesIndices[prevIndex]].site;
								cVec2 edgeToNext = (nxtVert - v.site).normalized();
								cVec2 edgeFromPrev = (v.site - prevVert).normalized();
								bool ccwNext = edgeDir.dot(-edgeToNext.tangent()) >= 0;
								bool ccwPrev = edgeDir.dot(-edgeFromPrev.tangent()) >= 0;
								if (ccwNext || ccwPrev) // if it doesnt keep CCW rotation for both the next or prev edge, it cannot be the correct edge and must be not an end vertex
								{
									if (infVIndexA < 0)
									{
										infVIndexA = i;
										infEIndexA = infEdge1;
									}
									else
									{
										infVIndexB = i;
										infEIndexB = infEdge1;
									}
								}
								
							}
							else // subcase 1 two infinite edge case
							{
								//		This means the finite edge HAS to make up the edge of the cell
								//		(else it means the cell has 2 infinite edges in 1 vertex, which is a degenerate vertex, filtered out earlier)
								//		meaning one of the two infinite edges is the actual edge of the cell, the other points away from the cell.
								//		If we take the inward facing tangent of the finite edge (pointing to the seed point),
								//		the infinite edge that has the highest dot product with the tangent is the edge of the cell
								int finiteEdgeIndex = -1;
								for (unsigned edgeIdx : v.edgeIndices) // go through its 3 edges
								{
									if (!edges[edgeIdx].infinite) // find only the finite edge
									{
										finiteEdgeIndex = edgeIdx;
										break;
									}
								}
								cVEdge finiteEdge = edges[finiteEdgeIndex]; // get the finite edge
								cVec2 tangent = (finiteEdge.endDir - finiteEdge.origin).tangent().normalized();
								if (tangent.dot(seed - finiteEdge.origin) < 0) // if the tangent points away from the seed, invert it
									tangent = -tangent;
								cVec2 edgeDir1 = edges[infEdge1].endDir.normalized(); // get the dir of the edge
								cVec2 edgeDir2 = edges[infEdge2].endDir.normalized(); // get the dir of the edge

								float dot1 = edgeDir1.dot(tangent);
								float dot2 = edgeDir2.dot(tangent);
								int finalIndex = (dot1 > dot2) ? infEdge1 : infEdge2;

								if (infVIndexA < 0)
								{
									infVIndexA = i;
									infEIndexA = finalIndex;
								}
								else
								{
									infVIndexB = i;
									infEIndexB = finalIndex;
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

	std::vector<cVec2> buildCell(const cVoronoiDiagram& pattern, cVCell& cell, float extensionFactor)
	{
		std::vector<cVec2> finalVerts;
		unsigned vertCount = cell.vertices.size();

		for (unsigned i = 0; i < vertCount; i++)
		{
			const cVVert& v = pattern.vertices[cell.vertices[i]];

			if (!cell.infinite)
			{
				finalVerts.push_back(v.site);
				continue;
			}

			if (vertCount == 1) // degen cell
			{
				// vert is the order of 
				// inf edge 1 , self, inf edge 2
				cVec2 extendedA = v.site, extendedB = v.site;
				bool hasA = false;
				for (unsigned i : v.edgeIndices)
				{
					if (pattern.edges[i].infinite)
					{
						if (!hasA)
						{
							extendedA += pattern.edges[i].endDir * extensionFactor;
							hasA = true;
						}
						else
							extendedB += pattern.edges[i].endDir * extensionFactor;
					}
				}
				finalVerts = { extendedA, v.site, extendedB };
				continue;
			}

			const cVVert& infVA = pattern.vertices[cell.vertices[cell.infVertA]];
			const cVVert& infVB = pattern.vertices[cell.vertices[cell.infVertB]];
			const cVEdge& infEA = pattern.edges[cell.infEdgeA];
			const cVEdge& infEB = pattern.edges[cell.infEdgeB];

			const cVec2 exA = infVA.site + (infEA.endDir.normalized() * extensionFactor);
			const cVec2 exB = infVB.site + (infEB.endDir.normalized() * extensionFactor);

			finalVerts.push_back(exA);          // Extended infinite edge A
			finalVerts.push_back(infVA.site);   // Infinite vertex A
			finalVerts.push_back(infVB.site);   // Infinite vertex B
			finalVerts.push_back(exB);          // Extended infinite edge B

			for (size_t j = 0; j < cell.vertices.size(); j++)
			{
				if (j != cell.infVertA && j != cell.infVertB)
				{
					finalVerts.push_back(pattern.vertices[cell.vertices[j]].site);
				}
			}

			const cVec2& seedPt = pattern.v_points[cell.seedIndex];
			std::sort(finalVerts.begin(), finalVerts.end(), [seedPt](const cVec2& a, const cVec2& b)
				{
					return atan2(a.y - seedPt.y, a.x - seedPt.x) < atan2(b.y - seedPt.y, b.x - seedPt.x);
				});
		}


		return finalVerts;
	}

	static float computePolygonArea(const std::vector<cVec2>& polygon) {
		if (polygon.size() < 3) return 0.0f;  // A polygon must have at least 3 vertices

		float area = 0.0f;
		size_t n = polygon.size();

		for (size_t i = 0; i < n; i++) {
			const cVec2& p1 = polygon[i];
			const cVec2& p2 = polygon[(i + 1) % n];  // Wraps around at the end
			area += (p1.x * p2.y - p2.x * p1.y);
		}

		return std::abs(area) * 0.5f;  // Return absolute value
	}


	std::vector<std::vector<cVec2>> ClipVoronoiWithPolygon(const cVoronoiDiagram& inPattern, const cVec2* p_vertices, const cVec2* p_normals, int p_count)
	{
		cTransform ixf; // identity
		cAABB bounds = CreateAABBHull(p_vertices, p_count, ixf);
		float extensionFactor = bounds.getExtents().sqrMagnitude() + 1; // we add 1 to avoid any `on the edge` issues for bounds with a sqrmag of 1 or 0
		std::vector<std::vector<cVec2>> clippedPolys;

		auto& cells = inPattern.getCells();

		for (auto& cell : cells)
		{
			std::vector<cVec2> poly = buildCell(inPattern, cell, extensionFactor * 2);
			std::vector<cVec2> ce = suther_land_hodgman(poly, { p_vertices, p_vertices + p_count });
			clippedPolys.push_back(ce);
		}

		std::sort(clippedPolys.begin(), clippedPolys.end(),
			[](const std::vector<cVec2>& a, const std::vector<cVec2>& b) {
				return computePolygonArea(a) > computePolygonArea(b);  // Largest to smallest
			});

		return clippedPolys;
	}
	

}
