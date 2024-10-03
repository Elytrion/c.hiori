#pragma once

#include "simplex.h"

namespace chiori
{
	class gjkobject
	{
	public:
		std::vector<vec2> vertices;
		vec2 position;
		vec2 velocity;

		gjkobject(std::vector<vec2> vertices, vec2 position = vec2::zero, vec2 velocity = vec2::zero) 
			: vertices(vertices), position(position), velocity(velocity) {}

		vec2 getSupportPoint(const vec2& inDir) const;

		vec2 getSupportPoint(const gjkobject& inOther, const vec2& inDir) const;
	};
	
	float gjk(const gjkobject& inPrimary, const gjkobject& inTarget, simplex& outSimplex);
}