#pragma once
#include <assert.h>

namespace chiori
{
	#define cassert(X) assert(X)

	using cint = int32_t;
	using cint16 = int16_t;
	using cunsigned = uint32_t;
	using cunsigned16 = uint16_t;
	using cfloat = float;
	
	namespace commons
	{
		inline int GJK_ITERATIONS = 32;
		inline int CTREE_START_CAPACITY = 32;
		inline float AABB_FATTEN_FACTOR = 0.1f; // This is used to fatten AABBs in the dynamic tree. 	
	}
}