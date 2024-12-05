#pragma once

#include "gjk.h"

namespace chiori
{	
	class CollisionPair
	{		
	public:
		CollisionPair(void* inPrimary, void* inTarget) : primary{ inPrimary }, target{ inTarget } {}
		
		void* primary;
		void* target;

		GJKresult collisionData;

		enum
		{
			ENTER,
			STAY,
			EXIT
		} state = ENTER;
	};
}
