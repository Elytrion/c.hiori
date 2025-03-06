#include "pch.h"
#include "voronoi.h"
#include "aabb.h"

namespace chiori
{
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
