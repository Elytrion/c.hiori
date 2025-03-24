#include "pch.h"
#include "geom.h"
#include "aabb.h"

namespace chiori
{
	struct cHull
	{
		cVec2 points[MAX_POLYGON_VERTICES];
		int count{ 0 };
	};

	// quickhull recursion
	static cHull qhRecurse(cVec2 p1, cVec2 p2, cVec2* ps, int32_t count)
	{
		cHull hull;
		hull.count = 0;

		if (count == 0)
		{
			return hull;
		}

		// create an edge vector pointing from p1 to p2
		cVec2 e = (p2 - p1).normalized();

		// discard points left of e and find point furthest to the right of e
		cVec2 rightPoints[MAX_POLYGON_VERTICES];
		int32_t rightCount = 0;

		int32_t bestIndex = 0;
		float bestDistance = cross((ps[bestIndex] - p1), e);
		if (bestDistance > 0.0f)
		{
			rightPoints[rightCount++] = ps[bestIndex];
		}

		for (int32_t i = 1; i < count; ++i)
		{
			float distance = cross((ps[i] - p1), e);
			if (distance > bestDistance)
			{
				bestIndex = i;
				bestDistance = distance;
			}

			if (distance > 0.0f)
			{
				rightPoints[rightCount++] = ps[i];
			}
		}

		if (bestDistance < 2.0f * commons::LINEAR_SLOP)
		{
			return hull;
		}

		cVec2 bestPoint = ps[bestIndex];

		// compute hull to the right of p1-bestPoint
		cHull hull1 = qhRecurse(p1, bestPoint, rightPoints, rightCount);

		// compute hull to the right of bestPoint-p2
		cHull hull2 = qhRecurse(bestPoint, p2, rightPoints, rightCount);

		// stitch together hulls
		for (int32_t i = 0; i < hull1.count; ++i)
		{
			hull.points[hull.count++] = hull1.points[i];
		}

		hull.points[hull.count++] = bestPoint;

		for (int32_t i = 0; i < hull2.count; ++i)
		{
			hull.points[hull.count++] = hull2.points[i];
		}

		cassert(hull.count < MAX_POLYGON_VERTICES);

		return hull;
	}

	// quickhull algorithm
	// - merges vertices based on s2_linearSlop
	// - removes collinear points using s2_linearSlop
	// - returns an empty hull if it fails
	cHull ComputeHull(const cVec2* points, int count)
	{
		cHull hull;
		hull.count = 0;

		if (count < 3 || count > MAX_POLYGON_VERTICES)
		{
			// check your data
			return hull;
		}

		count = c_min(count, MAX_POLYGON_VERTICES);

		cAABB aabb = { {FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX} };

		// Perform aggressive point welding. First point always remains.
		// Also compute the bounding box for later.
		cVec2 ps[MAX_POLYGON_VERTICES];
		int32_t n = 0;
		const float tolSqr = 16.0f * commons::LINEAR_SLOP * commons::LINEAR_SLOP;
		for (int32_t i = 0; i < count; ++i)
		{
			aabb.min = c_min(aabb.min, points[i]);
			aabb.max = c_max(aabb.max, points[i]);

			cVec2 vi = points[i];

			bool unique = true;
			for (int32_t j = 0; j < i; ++j)
			{
				cVec2 vj = points[j];

				float distSqr = distanceSqr(vi, vj);
				if (distSqr < tolSqr)
				{
					unique = false;
					break;
				}
			}

			if (unique)
			{
				ps[n++] = vi;
			}
		}

		if (n < 3)
		{
			// all points very close together, check your data and check your scale
			return hull;
		}

		// Find an extreme point as the first point on the hull
		cVec2 c = aabb.getCenter();
		int32_t f1 = 0;
		float dsq1 = distanceSqr(c, ps[f1]);
		for (int32_t i = 1; i < n; ++i)
		{
			float dsq = distanceSqr(c, ps[i]);
			if (dsq > dsq1)
			{
				f1 = i;
				dsq1 = dsq;
			}
		}

		// remove p1 from working set
		cVec2 p1 = ps[f1];
		ps[f1] = ps[n - 1];
		n = n - 1;

		int32_t f2 = 0;
		float dsq2 = distanceSqr(p1, ps[f2]);
		for (int32_t i = 1; i < n; ++i)
		{
			float dsq = distanceSqr(p1, ps[i]);
			if (dsq > dsq2)
			{
				f2 = i;
				dsq2 = dsq;
			}
		}

		// remove p2 from working set
		cVec2 p2 = ps[f2];
		ps[f2] = ps[n - 1];
		n = n - 1;

		// split the points into points that are left and right of the line p1-p2.
		cVec2 rightPoints[MAX_POLYGON_VERTICES - 2];
		int32_t rightCount = 0;

		cVec2 leftPoints[MAX_POLYGON_VERTICES - 2];
		int32_t leftCount = 0;

		cVec2 e = (p2 - p1).normalized();

		for (int32_t i = 0; i < n; ++i)
		{
			float d = cross((ps[i] - p1), e);

			// slop used here to skip points that are very close to the line p1-p2
			if (d >= 2.0f * commons::LINEAR_SLOP)
			{
				rightPoints[rightCount++] = ps[i];
			}
			else if (d <= -2.0f * commons::LINEAR_SLOP)
			{
				leftPoints[leftCount++] = ps[i];
			}
		}

		// compute hulls on right and left
		cHull hull1 = qhRecurse(p1, p2, rightPoints, rightCount);
		cHull hull2 = qhRecurse(p2, p1, leftPoints, leftCount);

		if (hull1.count == 0 && hull2.count == 0)
		{
			// all points collinear
			return hull;
		}

		// stitch hulls together, preserving CCW winding order
		hull.points[hull.count++] = p1;

		for (int32_t i = 0; i < hull1.count; ++i)
		{
			hull.points[hull.count++] = hull1.points[i];
		}

		hull.points[hull.count++] = p2;

		for (int32_t i = 0; i < hull2.count; ++i)
		{
			hull.points[hull.count++] = hull2.points[i];
		}

		cassert(hull.count <= MAX_POLYGON_VERTICES);

		// merge collinear
		bool searching = true;
		while (searching && hull.count > 2)
		{
			searching = false;

			for (int i = 0; i < hull.count; ++i)
			{
				int i1 = i;
				int i2 = (i + 1) % hull.count;
				int i3 = (i + 2) % hull.count;

				cVec2 s1 = hull.points[i1];
				cVec2 s2 = hull.points[i2];
				cVec2 s3 = hull.points[i3];

				// unit edge vector for s1-s3
				cVec2 r = (s3 - s1).normalized();

				float distance = cross((s2 - s1), r);
				if (distance <= 2.0f * commons::LINEAR_SLOP)
				{
					// remove midpoint from hull
					for (int j = i2; j < hull.count - 1; ++j)
					{
						hull.points[j] = hull.points[j + 1];
					}
					hull.count -= 1;

					// continue searching for collinear points
					searching = true;

					break;
				}
			}
		}

		if (hull.count < 3)
		{
			// all points collinear, shouldn't be reached since this was validated above
			hull.count = 0;
		}

		return hull;
	}
	

	void cPolygon::Set(const cVec2* inPoints, int inCount)
	{
		cassert(inCount > 2 && inCount <= MAX_POLYGON_VERTICES);

		cHull hull = ComputeHull(inPoints, inCount);
		
		// Copy vertices.
		for (int i = 0; i < hull.count; ++i)
			vertices[i] = hull.points[i];
		count = hull.count;
		// Compute normals. Ensure the edges have non-zero length.
		for (int i = 0; i < count; ++i)
		{
			int j = (i + 1) % count;
			cVec2 edge = vertices[j] - vertices[i];
			cassert(edge.sqrMagnitude() > EPSILON * EPSILON);
			normals[i] = cross(edge, 1.0f);
			normals[i].normalize();
		}
	}

	cPolygon::cPolygon(const cVec2* inPoints, int inCount)
	{
		Set(inPoints, inCount);
	}

	cPolygon GeomMakeRegularPolygon(int count) {
		cassert(count >= 3 && count <= MAX_POLYGON_VERTICES);

		cVec2 vertices[MAX_POLYGON_VERTICES];
		float angleStep = 2 * PI / count;

		for (int i = 0; i < count; ++i) {
			float angle = i * angleStep;
			vertices[i] = { cos(angle), sin(angle) };
		}

		return cPolygon(vertices, count);
	}

	cPolygon GeomMakeBox(float hx, float hy) {
		cVec2 halfExtents = { hx / 2, hy / 2 };
		cPolygon poly;
		
		poly.count = 4;
		poly.vertices[0] = { -hx, -hy };
		poly.vertices[1] = { hx, -hy };
		poly.vertices[2] = { hx, hy };
		poly.vertices[3] = { -hx, hy };
		poly.normals[0] = { 0.0f, -1.0f };
		poly.normals[1] = { 1.0f, 0.0f };
		poly.normals[2] = { 0.0f, 1.0f };
		poly.normals[3] = { -1.0f, 0.0f };
		poly.radius = 0.0f;

		return poly;
	}

	cPolygon GeomMakeBox(cVec2 min, cVec2 max)
	{
		cPolygon poly;

		poly.count = 4;
		poly.vertices[0] = { min.x, min.y };
		poly.vertices[1] = { max.x, min.y };
		poly.vertices[2] = { max.x, max.y };
		poly.vertices[3] = { min.x, max.y };
		poly.normals[0] = { 0.0f, -1.0f };
		poly.normals[1] = { 1.0f, 0.0f };
		poly.normals[2] = { 0.0f, 1.0f };
		poly.normals[3] = { -1.0f, 0.0f };
		poly.radius = 0.0f;
		
		return poly;
	}

	cPolygon GeomMakeSquare(float h) {
		return GeomMakeBox(h, h);
	}

	cPolygon GeomMakeOffsetBox(float hx, float hy, cVec2 center, float angleRadians)
	{
		cTransform xf;
		xf.p = center;
		xf.q.set(angleRadians);

		cPolygon poly;
		poly.count = 4;
		poly.vertices[0] = cTransformVec(xf, { -hx, -hy });
		poly.vertices[1] = cTransformVec(xf, { hx, -hy });
		poly.vertices[2] = cTransformVec(xf, { hx, hy });
		poly.vertices[3] = cTransformVec(xf, { -hx, hy });
		poly.normals[0] = cVec2::down.rotated(xf.q);
		poly.normals[1] = cVec2::right.rotated(xf.q);
		poly.normals[2] = cVec2::up.rotated(xf.q);
		poly.normals[3] = cVec2::left.rotated(xf.q);
		poly.radius = 0.0f;
		return poly;
	}

	cMassData cPolygon::ComputeMass(float density) const
	{
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		cassert(count > 0);

		cVec2 c_vertices[MAX_POLYGON_VERTICES];

		if (radius > 0.0f)
		{
			// Push out vertices according to radius. This improves
			// the mass accuracy, especially the rotational inertia.
			for (int i = 0; i < count; ++i)
			{
				int j = i == 0 ? count - 1 : i - 1;
				cVec2 n1 = normals[j];
				cVec2 n2 = normals[i];

				cVec2 mid = (n1 + n2).normalized();
				cVec2 t1 = { -n1.y, n1.x };
				float sinHalfAngle = cross(mid, t1);

				float offset = radius;
				if (sinHalfAngle > FLT_EPSILON)
				{
					offset = radius / sinHalfAngle;
				}

				c_vertices[i] = vertices[i] + (offset * mid);
			}
		}
		else
		{
			for (int32_t i = 0; i < count; ++i)
			{
				c_vertices[i] = vertices[i];
			}
		}

		cVec2 center = { 0.0f, 0.0f };
		float area = 0.0f;
		float I = 0.0f;

		// Get a reference point for forming triangles.
		// Use the first vertex to reduce round-off errors.
		cVec2 r = vertices[0];

		const float inv3 = 1.0f / 3.0f;

		for (int32_t i = 1; i < count - 1; ++i)
		{
			// Triangle edges
			cVec2 e1 = (vertices[i] - r);
			cVec2 e2 = (vertices[i + 1] - r);

			float D = cross(e1, e2);

			float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid, r at origin
			center += (triangleArea * inv3 * (e1 + e2));

			float ex1 = e1.x, ey1 = e1.y;
			float ex2 = e2.x, ey2 = e2.y;

			float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
			float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

			I += (0.25f * inv3 * D) * (intx2 + inty2);
		}

		cMassData massData;

		// Total mass
		massData.mass = density * area;

		// Center of mass, shift back from origin at r
		cassert(area > FLT_EPSILON);
		float invArea = 1.0f / area;
		center.x *= invArea;
		center.y *= invArea;
		massData.center = (r + center);

		// Inertia tensor relative to the local origin (point s).
		massData.I = density * I;

		// Shift to center of mass then to original body origin.
		massData.I += massData.mass * (dot(massData.center, massData.center) - dot(center, center));

		return massData;
	}
}