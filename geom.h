#pragma once
#include "chioriMath.h"

namespace chiori
{
	#define MAX_POLYGON_VERTICES 8

	/// This holds the mass data computed for a shape.
	struct cMassData
	{
		/// The mass of the shape, usually in kilograms.
		float mass;

		/// The position of the shape's centroid relative to the shape's origin.
		cVec2 center;

		/// The rotational inertia of the shape about the local origin.
		float I;
	};

	
	/// A solid convex polygon. It is assumed that the interior of the polygon is to
	/// the left of each edge.
	/// Polygons have a maximum number of vertices equal to MAX_POLYGON_VERTICES.
	/// In most cases you should not need many vertices for a convex polygon. Even 16 feels generous
	struct cPolygon
	{
		cVec2 vertices[MAX_POLYGON_VERTICES];	// the untransformed vertices of the shape (assumes shape is centered at 0,0 with no scale nor rotation)
		cVec2 normals[MAX_POLYGON_VERTICES];		// the normals of all the faces of the shape
		int count{ -1 };						// the number of vertices/normals
		float radius{ 0.0f };					// for curved shapes (TODO: not implemented yet)
		
		cPolygon() : count{ 0 }, radius{ 0.0f } {};
		cPolygon(const cVec2* points, int count);

		void Set(const cVec2* points, int count);
		
		cMassData ComputeMass(float density) const;

		int GetCount() const { return count; }
		const cVec2& GetVertex(int index) const { return vertices[index]; }
		const cVec2& GetNormal(int index) const { return normals[index]; }
	};
	
	// helper functions
	cPolygon GeomMakeRegularPolygon(const cVec2* points, int count);
	cPolygon GeomMakeSquare(float h);
	cPolygon GeomMakeBox(float hx, float hy);
	cPolygon GeomMakeOffsetBox(float hx, float hy, cVec2 center, float angle = 0.0f);
}