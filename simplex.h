#pragma once

#include "pch.h"
#include "vector2.h"
#include <array>

namespace chiori
{
	struct MVert // Minkowski Vertex, a vertex on the Configuration Space Object (CSO)
	{
		vec2 a{ 0,0 }; // support point on obj A
		vec2 b{ 0,0 }; // support point on obj B
		vec2 w{ 0,0 }; // support point on CSO aka minkowski difference between points a and b (a - b)
		
		MVert(vec2 inA = vec2::zero, vec2 inB = vec2::zero) : a(inA), b(inB), w(inA - inB) {}
	};
}