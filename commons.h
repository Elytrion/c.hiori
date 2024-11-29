#pragma once
#include <assert.h>

namespace chiori
{
	#define cassert(X) assert(X)
	namespace commons
	{
		inline int GJK_ITERATIONS = 24;
		inline int CTREE_START_CAPACITY = 32;
		inline float AABB_FATTEN_FACTOR = 1.0f; // This is used to fatten AABBs in the dynamic tree. 
	}
}