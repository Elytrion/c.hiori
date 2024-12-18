#pragma once
#include <assert.h>

namespace chiori
{
	#define cassert(X) assert(X)

	#ifndef DEFAULT_ALLOCATOR
	#define DEFAULT_ALLOCATOR std::allocator<char>
	#endif

	namespace commons
	{
		inline int GJK_ITERATIONS = 32;
		inline int CTREE_START_CAPACITY = 32;
		inline float AABB_FATTEN_FACTOR = 1.0f; // This is used to fatten AABBs in the dynamic tree. 
		
	}
}