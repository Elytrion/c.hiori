#pragma once

#include "geom.h"
#include "aabb.h"
#include "flag.h"

namespace chiori
{
	struct ShapeConfig
	{
		float friction{ 0.5f };
		float restitution{ 0.0f };
		float density{ 1.0f };
	};
	
	class cShape
	{
	public:
		enum // Shape flags
		{
			SCENE_QUERYABLE = (1 << 0), // this shape can be queried (either by raycasts or triggers)
			IS_TRIGGER = (1 << 1),		// this shape is a trigger and will not participate in collision response
			IS_STATIC = (1 << 2)		// this shape will not move during simulation time (set for broadphase optimization)
		};

		int actorIndex{ -1 };		// the index of the actor that holds this shape
		int nextShapeIndex{ -1 };	// the index of the next shape in the actor's shape linked list
		int broadphaseIndex{ -1 };	// the index of this shape in the broadphase structure
		
		cPolygon polygon;			// holds the vertices and normals of the shape
		
		float friction{ 0.5f };
		float restitution{ 0.1f };
		float density{ 1.0f };
		
		AABB aabb;						// untransformed close fit AABB
		Flag_8 shapeFlags = SCENE_QUERYABLE;
		
		void* userData{ nullptr }; 			// to hold a pointer to any user specific data (user holds ownership of data)

		cShape() {};
		cShape(const vec2* inVertices, int count) { polygon.Set(inVertices, count); }
		int getCount() const { return polygon.count; }
		void setVertices(const vec2* inVertices, int count) { polygon.Set(inVertices, count); }
		const vec2* getBaseVertices() const { return polygon.vertices; }
		void getVertices(vec2* inVertices, cTransform xf) const;
		cMassData computeMass() const { return polygon.ComputeMass(density); }
		
		AABB ComputeAABB(const cTransform& xf)
		{
			return CreateAABBHull(polygon.vertices, polygon.count, xf);
		}	

		bool operator==(const cShape& inRHS) const {
			return this == &inRHS;
		}
	};
	
	// this function assumes the size of the array passed in is equal to or greater than count
	inline void cShape::getVertices(vec2* inVertices, cTransform xf) const
	{
		int count = polygon.count;
		const vec2* verts = polygon.vertices;
		for (int i = 0; i < count; ++i)
		{
			inVertices[i] = verts[i];
			inVertices[i].x *= xf.scale.x;
			inVertices[i].y *= xf.scale.y;
			inVertices[i].rotate(xf.rot);
			inVertices[i] += xf.pos;
		} 
	}
}