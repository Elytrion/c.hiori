#include "pch.h"
#include "geom.h"

namespace chiori
{
	void cPolygon::Set(const cVec2* inPoints, int inCount)
	{
		cassert(inCount > 2 && inCount <= MAX_POLYGON_VERTICES);
		count = inCount;

		// Copy vertices.
		for (int i = 0; i < count; ++i)
			vertices[i] = inPoints[i];

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

	cPolygon GeomMakeRegularPolygon(const cVec2* points, int count) {
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