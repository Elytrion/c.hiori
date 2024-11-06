#include "pch.h"
#include "physicsWorld.h"

namespace chiori
{
	void PhysicsWorld::update(float inDT)
	{
		// Accumulate the time since the last frame
		accumulator += inDT;
		
		while (accumulator >= physicsStepTime) {
			simulate(physicsStepTime); // Run your fixed-time physics updates
			accumulator -= physicsStepTime;
		}
	}

	void PhysicsWorld::simulate(float inDT)
	{
		for (cActor& a : actors)
		{
			if (a.getFlags().isSet(cActor::IS_STATIC))
				continue;
			a.integrate(gravity, inDT);
		}
	}

	cActor& PhysicsWorld::AddActor(const std::vector<vec2>& vertices)
	{
		actors.emplace_back(vertices);
		return actors.back();
	}

	void PhysicsWorld::RemoveActor(cActor& inActor)
	{
		actors.erase(std::remove(actors.begin(), actors.end(), inActor), actors.end());
	}
	
}