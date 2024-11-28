#pragma once

#include "cActor.h"
#include "broadphase.h"

namespace chiori
{
	class PhysicsWorld
	{
	private:
		float accumulator = 0.0f;
		std::vector<cActor*> actors;

	public:		
		~PhysicsWorld();
		
		float physicsStepTime = 0.0167f;
		vec2 gravity = { 0.0f, 9.81f };
		void update(float inDT);	// converts update into fixed updates
		void simulate(float inDT);	// simulates one time step of physics, call directly if not using update

		cActor* AddActor(const std::vector<vec2>& vertices);
		void RemoveActor(cActor* inActor);
		std::vector<cActor*>& getWorldActors() { return actors; }
	};
}