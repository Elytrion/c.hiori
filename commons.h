#pragma once
#include <assert.h>

namespace chiori
{
	#define cassert(X) assert(X)
	namespace commons
	{
		constexpr float EPSILON = 1e-5f;		// Epislon value for comparing floats
		constexpr float LEPSILON = 1e-3f;		// Low precision epsilon value
		constexpr float HEPSILON = 1e-7f;		// High precision epsilon value
		constexpr float PI = 3.14159265f;
		constexpr float DEG2RAD = PI / 180.0f;
		constexpr float RAD2DEG = 180.0f / PI; 
		inline int GJK_ITERATIONS = 24;
		inline int CTREE_START_CAPACITY = 32;
		inline float AABB_FATTEN_FACTOR = 1.0f; // This is used to fatten AABBs in the dynamic tree. 
	}
}