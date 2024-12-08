#pragma once

#include "chioriMath.h"
#include "aabb.h"
#include "flag.h"

namespace chiori
{
	class PhysicsWorld; // forward declaration
	
	class cShape
	{
	private:
		friend class PhysicsWorld;	// the physics world will be able to access all internal values as required, while users cannot and should not touch them
		std::vector<vec2> vertices; // the untransformed vertices of the shape (assumes shape is centered at 0,0 with no scale nor rotation)
		std::vector<vec2> normals;  // the normals of all the faces of the shape
		int actorIndex = -1;		// the index of the actor that holds this shape
		int broadphaseIndex = -1;	// the index of this shape in the broadphase structure
		
	public:
		enum // Shape flags
		{
			SCENE_QUERYABLE = (1 << 0), // this shape can be queried (either by raycasts or triggers)
			IS_TRIGGER = (1 << 1),		// this shape is a trigger and will not participate in collision response
			IS_STATIC = (1 << 2)		// this shape will not move during simulation time (set for broadphase optimization)
		};
		
		float friction;
		float restitution;
		AABB aabb;						// untransformed close fit AABB
		Flag_8 shapeFlags = SCENE_QUERYABLE;
		
		void* userData; 			// to hold a pointer to any user specific data (user holds ownership of data)

		cShape() {};
		cShape(const std::vector<vec2>& inVertices);
	};

	inline cShape::cShape(const std::vector<vec2>& inVertices)
	{
		cassert(inVertices.size() >= 3);
		// copy over the data
		vertices = inVertices;
		// build AABB
		aabb = CreateAABBHull(vertices.data(), vertices.size()); 
		// calculate normals
		for (int i = 0; i < vertices.size(); ++i)
		{
			int i1 = i;
			int i2 = i + 1 % vertices.size();
			vec2 edge = vertices[i2] - vertices[i1];
			cassert(edge.sqrMagnitude() > EPSILON * EPSILON);
			normals[i] = edge.scross(1.0f).normalize();
		}
	}
}