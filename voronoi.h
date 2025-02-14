#pragma once
#include "delaunator.hpp"
#include <unordered_map>


namespace chiori
{
	struct cVEdge
	{
		cVec2 origin;			// origin of the edge
		cVec2 endDir;			// if infinite, endDir is the direction of the edge, otherwise it is the end point
		bool infinite{ false }; // if true, the edge is infinite
		cVEdge(const cVec2& inOrigin = cVec2::zero, const cVec2& inEndDir = cVec2::zero, bool inInfinite = false) :
			origin{ inOrigin }, endDir{ inEndDir }, infinite{ inInfinite } {}
	};

	struct cVCell
	{
		cVec2 site;					// the centriod/site of the cell
		std::vector<cVEdge> edges;	// the edges that make up this cell
	};

	class cVoronoiDiagram
	{
	public:
		std::vector<cVCell> cells; // Each cell corresponds to a Voronoi region

		void create(const cVec2* points, unsigned count)
		{
			std::vector<cVec2> pointsVec(points, points + count);
			std::vector<std::vector<cVec2>> triangles = triangulateDelaunator(pointsVec);
			
			// Map each site (Delaunay point) to its Voronoi edges
			std::unordered_map<cVec2, cVCell, cVec2Hash> voronoiCells;

			// Compute circumcenters and construct Voronoi edges
			std::unordered_map<size_t, cVec2> circumcenters;
			for (size_t i = 0; i < triangles.size(); i++) {
				cVec2 a = triangles[i][0];
				cVec2 b = triangles[i][1];
				cVec2 c = triangles[i][2];

				cVec2 circum = circumcenter(a, b, c);
				circumcenters[i] = circum;
			}

			// Generate Voronoi edges by connecting circumcenters of adjacent triangles
			for (size_t i = 0; i < triangles.size(); i++)
			{
				for (size_t j = 0; j < 3; j++)
				{
					size_t edgeStart = j;
					size_t edgeEnd = (j + 1) % 3;

					size_t t0 = i;
					size_t t1 = findAdjacentTriangle(i, edgeStart, edgeEnd, triangles);

					if (t1 != std::numeric_limits<size_t>::max()) {
						voronoiCells[triangles[i][edgeStart]].edges.push_back(cVEdge(circumcenters[t0], circumcenters[t1]));
					}
					else {
						// Infinite edge handling (convex hull edges)
						cVec2 midpoint((triangles[i][edgeStart].x + triangles[i][edgeEnd].x) / 2,
							(triangles[i][edgeStart].y + triangles[i][edgeEnd].y) / 2);
						cVec2 dir = { circumcenters[t0].x - midpoint.x, circumcenters[t0].y - midpoint.y };
						voronoiCells[triangles[i][edgeStart]].edges.push_back(cVEdge(circumcenters[t0], dir, true));
					}
				}
			}

			// Convert map to vector
			for (auto& pair : voronoiCells) {
				pair.second.site = centriod(pair.second);
				cells.push_back(pair.second);
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
	
		cVec2 centriod(cVCell cell)
		{
			float sumX = 0, sumY = 0;
			size_t count = cell.edges.size();

			for (const auto& v : cell.edges) {
				sumX += v.origin.x;
				sumY += v.origin.y;
			}

			return cVec2(sumX / count, sumY / count);
		}
	};
}