#pragma once
#include "delaunator.hpp"


namespace chiori
{
	struct cVEdge
	{
		cVec2 origin;			// origin of the edge
		cVec2 endDir;			// if infinite, endDir is the direction of the edge, otherwise it is the end point
		bool infinite{ false }; // if true, the edge is infinite
		cVEdge(const cVec2& inOrigin = cVec2::zero, const cVec2& inEndDir = cVec2::zero, bool inInfinite = false) :
			origin{ inOrigin }, endDir{ inEndDir }, infinite{ inInfinite } {}

		bool operator==(const cVEdge& other) const { return origin == other.origin && endDir == other.endDir; }
	};

	struct cVVert
	{
		cVec2 site;							// the centriod/site of the vertex
		std::vector<unsigned> edgeIndices;	// the indices of the edges that make up this vertex
	};


	class cVoronoiDiagram
	{
	public:
		std::vector<cVVert> vertices; // Each vertex corresponds to a Voronoi region
		std::vector<cVEdge> edges; // All unique edges in the Voronoi diagram

		void create(const cVec2* points, unsigned count)
		{
			std::vector<cVec2> pointsVec(points, points + count);
			std::vector<std::vector<cVec2>> triangles = triangulateDelaunator(pointsVec);

			std::vector<cVec2> circumcenters;
			cVec2 voronoiCentroid(0, 0);
			size_t cc_count = 0;

			for (size_t i = 0; i < triangles.size(); i++) {
				cVec2 circum = circumcenter(triangles[i][0], triangles[i][1], triangles[i][2]);
				circumcenters.push_back(circum);
				voronoiCentroid += circum;
				cc_count++;
			}

			if (cc_count > 0) {
				voronoiCentroid /= cc_count;
			}
			std::unordered_map<cVec2, std::vector<unsigned>, cVec2Hash> siteToEdges;
			for (size_t i = 0; i < triangles.size(); i++) {
				cVVert vertex;
				vertex.site = circumcenters[i];

				for (size_t j = 0; j < 3; j++) {
					size_t edgeStart = j;
					size_t edgeEnd = (j + 1) % 3;

					size_t t0 = i;
					size_t t1 = findAdjacentTriangle(i, edgeStart, edgeEnd, triangles);

					cVEdge edge;
					if (t1 != std::numeric_limits<size_t>::max()) {
						edge = cVEdge(circumcenters[t0], circumcenters[t1]);
					}
					else {
						cVec2 midpoint = (triangles[i][edgeStart] + triangles[i][edgeEnd]) * 0.5f;
						cVec2 dir = (circumcenters[t0] - midpoint);

						if (dir.dot(voronoiCentroid - midpoint) > 0) {
							dir = -dir;
						}

						edge = cVEdge(circumcenters[t0], dir, true);
					}

					auto itr = std::find_if(edges.begin(), edges.end(), [&](const cVEdge& e) {
						return e == edge;
						});
					int edgeIndex = -1;
					if (itr != edges.end()) {
						edgeIndex = std::distance(edges.begin(), itr);
					}
					else
					{
						edges.push_back(edge);
						edgeIndex = edges.size() - 1;
					}
					vertex.edgeIndices.push_back(edgeIndex);

					siteToEdges[triangles[i][edgeStart]].push_back(edgeIndex);
					siteToEdges[triangles[i][edgeEnd]].push_back(edgeIndex);
				}

				vertices.push_back(vertex);
			}
		}

		static inline std::vector<std::vector<cVec2>> triangulateDelaunator(const std::vector<cVec2>& points)
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
	private:
		cVec2 circumcenter(cVec2 a, cVec2 b, cVec2 c)
		{
			float dA = a.dot(a);
			float dB = b.dot(b);
			float dC = c.dot(c);
			float det = 2.0f * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
			return { (dA * (b.y - c.y) + dB * (c.y - a.y) + dC * (a.y - b.y)) / det,
								 (dA * (c.x - b.x) + dB * (a.x - c.x) + dC * (b.x - a.x)) / det };
		}

		size_t findAdjacentTriangle(size_t current, size_t edgeStart, size_t edgeEnd, const std::vector<std::vector<cVec2>>& triangles)
		{
			for (size_t i = 0; i < triangles.size(); i++) {
				if (i == current) continue;
				for (size_t j = 0; j < 3; j++) {
					if ((triangles[i][j] == triangles[current][edgeEnd] && triangles[i][(j + 1) % 3] == triangles[current][edgeStart]) ||
						(triangles[i][(j + 1) % 3] == triangles[current][edgeEnd] && triangles[i][j] == triangles[current][edgeStart])) {
						return i;
					}
				}
			}
			return std::numeric_limits<size_t>::max();
		}
	
		cVec2 centriod(cVVert cell)
		{
			float sumX = 0, sumY = 0;
			size_t count = cell.edgeIndices.size();

			for (const auto& ei : cell.edgeIndices) {
				const cVEdge& v = edges[ei];
				sumX += v.origin.x;
				sumY += v.origin.y;
			}

			return cVec2(sumX / count, sumY / count);
		}
	};

}