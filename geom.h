#pragma once
#include "chioriMath.h"

namespace chiori
{
	#define MAX_POLYGON_VERTICES 16

	/// The precursor to creating a polygon
	/// Will be used to create a valid polygon
	struct cHull
	{
		vec2 points[MAX_POLYGON_VERTICES];
		int count{ -1 };
	};
	
	/// A solid convex polygon. It is assumed that the interior of the polygon is to
	/// the left of each edge.
	/// Polygons have a maximum number of vertices equal to MAX_POLYGON_VERTICES.
	/// In most cases you should not need many vertices for a convex polygon. Even 16 feels generous
	struct cPolygon
	{
		vec2 vertices[MAX_POLYGON_VERTICES];	// the untransformed vertices of the shape (assumes shape is centered at 0,0 with no scale nor rotation)
		vec2 normals[MAX_POLYGON_VERTICES];		// the normals of all the faces of the shape
		int count{ -1 };						// the number of vertices/normals
		float radius{ 0.0f };					// for curved shapes (TODO: not implemented yet)
	};
	
	cHull ComputeHull(const vec2* points, int count);
	bool ValidateHull(const cHull* hull);
	

	cPolygon GeomMakePolygon(const cHull* hull);
	cPolygon GeomMakeSquare(float h);
	cPolygon GeomMakeBox(float hx, float hy);
	cPolygon GeomMakeOffsetBox(float hx, float hy, vec2 center, float angle);

	
	/// This holds the mass data computed for a shape.
	struct cMassData
	{
		/// The mass of the shape, usually in kilograms.
		float mass;

		/// The position of the shape's centroid relative to the shape's origin.
		vec2 center;

		/// The rotational inertia of the shape about the local origin.
		float I;
	};
	
	cMassData ComputePolygonMass(const cPolygon* shape, float density);
	
}