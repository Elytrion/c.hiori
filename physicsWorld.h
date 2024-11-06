#pragma once

#include "vector2.h"
#include "aabbtree.h"
#include "cActor.h"

namespace chiori
{
	class PhysicsWorld
	{
	private:
		float accumulator = 0.0f;
		std::vector<cActor> actors; //TODO: replace with aabb tree

	public:
		float physicsStepTime = 0.0167f;
		vec2 gravity = { 0.0f, 9.81f };
		void update(float inDT);	// converts update into fixed updates
		void simulate(float inDT);	// simulates one time step of physics, call directly if not using update

		cActor& AddActor();
		void RemoveActor(cActor& inActor);
		const std::vector<cActor>& getWorldActors() { return actors; }
	};
}