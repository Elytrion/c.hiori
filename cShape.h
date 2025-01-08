#pragma once

#include "chioriMath.h"
#include "aabb.h"
#include "flag.h"

namespace chiori
{
	struct ShapeConfig
	{
		float friction{ 0.5f };
		float restitution{ 0.0f };
		std::vector<vec2> vertices;
	};

	struct cPolygon
	{
		std::vector<vec2> vertices; // the untransformed vertices of the shape (assumes shape is centered at 0,0 with no scale nor rotation)
		std::vector<vec2> normals;  // the normals of all the faces of the shape
		int count {-1};				// the number of vertices/normals
		float radius{ 0.0f };		// for curved shapes (TODO: not implemented yet)
	};
	
	class cShape
	{
	public:
		int actorIndex = -1;		// the index of the actor that holds this shape
		int broadphaseIndex = -1;	// the index of this shape in the broadphase structure
		
		cPolygon polygon;			// holds the vertices and normals of the shape
		enum // Shape flags
		{
			SCENE_QUERYABLE = (1 << 0), // this shape can be queried (either by raycasts or triggers)
			IS_TRIGGER = (1 << 1),		// this shape is a trigger and will not participate in collision response
			IS_STATIC = (1 << 2)		// this shape will not move during simulation time (set for broadphase optimization)
		};
		float friction{ 0.5f };
		float restitution{ 0.1f };
		AABB aabb;						// untransformed close fit AABB
		Flag_8 shapeFlags = SCENE_QUERYABLE;
		void* userData{ nullptr }; 			// to hold a pointer to any user specific data (user holds ownership of data)

		cShape() {};
		cShape(const std::vector<vec2>& inVertices) { setVertices(inVertices); }
		void setVertices(const std::vector<vec2>& inVertices);
		std::vector<vec2> getVertices(cTransform inTfm);

		int GetActorIndex() { return actorIndex; }

		bool operator==(const cShape& inRHS) const {
			return this == &inRHS;
		}
	};

	inline void cShape::setVertices(const std::vector<vec2>& inVertices)
	{
		cassert(inVertices.size() >= 3);
		// copy over the data
		std::vector<vec2>& vertices = polygon.vertices;
		std::vector<vec2>& normals = polygon.normals;
		int& count = polygon.count;
		vertices = inVertices;
		count = static_cast<int>(inVertices.size());
		// calculate normals
		for (int i = 0; i < count; ++i)
		{
			int i1 = i;
			int i2 = (i + 1) % count;
			vec2 edge = vertices[i2] - vertices[i1];
			cassert(edge.sqrMagnitude() > EPSILON * EPSILON);
			normals.push_back(edge.scross(1.0f).normalize());
		}
	}

	inline std::vector<vec2> cShape::getVertices(cTransform inTfm)
	{
		std::vector<vec2> n_verts = polygon.vertices;
		// Apply position, scale and rotation to base vertices
		for (auto& vertex : n_verts)
		{
			vertex.x = vertex.x * inTfm.scale.x;
			vertex.y = vertex.y * inTfm.scale.y;
			vertex = vertex.rotate(inTfm.rot);
			vertex = vertex + inTfm.pos;
		}
		return n_verts;
	}
}