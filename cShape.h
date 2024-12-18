#pragma once

#include "chioriMath.h"
#include "aabb.h"
#include "flag.h"

namespace chiori
{
	class PhysicsWorld; // forward declaration

	struct cPolygon
	{
		std::vector<vec2> vertices; // the untransformed vertices of the shape (assumes shape is centered at 0,0 with no scale nor rotation)
		std::vector<vec2> normals;  // the normals of all the faces of the shape
		int count;					// the number of vertices/normals

		void setVertices(const std::vector<vec2>& inVertices)
		{
			cassert(inVertices.size() >= 3);
			// copy over the data
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
	};
	
	class cShape
	{
	private:
		friend class PhysicsWorld;	// the physics world will be able to access all internal values as required, while users cannot and should not touch them
		int actorIndex = -1;		// the index of the actor that holds this shape
		int broadphaseIndex = -1;	// the index of this shape in the broadphase structure
		
	public:
		cPolygon polygon;			// holds the vertices and normals of the shape
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
		cShape(const std::vector<vec2>& inVertices) { setVertices(inVertices); }
		void setVertices(const std::vector<vec2>& inVertices);
		std::vector<vec2> getVertices(cTransform inTfm);

		float inertia(float mass, const vec2& scale, const vec2& comOffset = vec2::zero);

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
		// build AABB
		aabb = CreateAABBHull(vertices.data(), vertices.size()); 
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

	inline float cShape::inertia(float mass, const vec2& scale, const vec2& comOffset) 
	{
		//// https://stackoverflow.com/questions/31106438/calculate-moment-of-inertia-given-an-arbitrary-convex-2d-polygon
		//cassert(mass > FLT_EPSILON);
		//float area = 0.0f;
		//vec2 center = vec2::zero;
		//float mmoi = 0.0f;
		//int n = vertices.size();

		//for (int i = 0; i < n; i++) // we calculate all the MOI for all the triangles in the convex shape
		//{
		//	const vec2& a = (vertices[i] * scale.x);
		//	const vec2& b = (vertices[(i + 1) % n] * scale.y);

		//	// Compute step values
		//	float area_step = a.cross(b) / 2.0f;
		//	vec2 center_step = (a + b) / 3.0f;
		//	float mmoi_step =
		//		area_step * (a.dot(a) + b.dot(b) + a.dot(b)) / 6.0f;

		//	// Accumulate values
		//	center = (center * area + center_step * area_step) / (area + area_step);
		//	area += area_step;
		//	mmoi += mmoi_step;
		//}
		//// Density is calculated from mass and total area
		//float density = mass / area;
		//// Scale mmoi by density and adjust to the center of mass
		//mmoi *= density;
		//mmoi -= mass * center.dot(center); // Parallel axis theorem adjustment
		//return mmoi;

		// sqr inertia tensor for simplicity until i can get ^ to behave
		cassert(mass > FLT_EPSILON);
		float mmoi = mass / 2.0f;
		vec2 sqrextents = aabb.getExtents().cmult(aabb.getExtents());
		mmoi = mmoi * (sqrextents.x + sqrextents.y);
		float d = mass * comOffset.sqrMagnitude();
		return (d + mmoi / 100.0f); // TODO: why divide by 100?
	}
}