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

	// Sutherland-Hodgman clipping
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


	/*static std::vector<cVec2> SutherlandHodgmanClipEdge(
		const cVec2& l1, const cVec2& l2, const cVec2* polyVerts, const cVec2* polyNorms, int polyCount)
	{
		std::vector<cVec2> output{l1, l2};
		for (int i = 0; i < polyCount; ++i)
		{
			cVec2 clipEdgeStart = polyVerts[i];
			cVec2 clipEdgeEnd = polyVerts[(i + 1) % polyCount];

			std::vector<cVec2> input = output;
			output.clear();

			for (int j = 0; j < input.size(); ++j)
			{
				cVec2 currPt = input[j];
				int index = (j - 1 + input.size()) % input.size();
				cVec2 prevPt = input[index];
				
				bool currInside = (polyNorms[i].dot(currPt - clipEdgeStart) <= 0);
				bool prevInside = (polyNorms[i].dot(prevPt - clipEdgeStart) <= 0);
				
				if (currInside)
				{
					if (!prevInside)
						output.push_back(getLineIntersection(clipEdgeStart, clipEdgeEnd, prevPt, currPt));
					output.push_back(currPt);
				}
				else if (prevInside)
				{
					output.push_back(getLineIntersection(clipEdgeStart, clipEdgeEnd, prevPt, currPt));
				}
			}
		}
		return output;
	}*/

	std::vector<cVec2> ClipVoronoiWithPolygon(const cVoronoiDiagram& inPattern, const cVec2* p_vertices, const cVec2* p_normals, int p_count)
	{
		cTransform ixf; // identity
		cAABB bounds = CreateAABBHull(p_vertices, p_count, ixf);
		float extensionFactor = bounds.getExtents().sqrMagnitude() + 1;
		std::unordered_set<cVec2, cVec2Hash> verts;
		for (const auto& edge : inPattern.edges)
		{
			if (edge.infinite)
				continue;
			
			cVec2 o = edge.origin;
			cVec2 e = /*(edge.infinite) ? (o + edge.endDir * extensionFactor) : */edge.endDir;

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
