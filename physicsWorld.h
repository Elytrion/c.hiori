#pragma once

#include "cActor.h"
#include "cShape.h"
#include "broadphase.h"
#include "chioriPool.h"
#include "commons.h"

namespace chiori
{
	class PhysicsWorld
	{
	private:
		using Allocator = DEFAULT_ALLOCATOR;
		cPool<cActor, Allocator> p_actors;
		cPool<cShape, Allocator> p_shapes;
		
		struct aPair
		{
			aPair(cActor* ina, cActor* inb) : a{ ina }, b{ inb } {};
			cActor* a;
			cActor* b;
		};
		
		float accumulator = 0.0f;
		std::vector<cActor*> m_actors;
		Broadphase m_broadphase;
		std::vector<aPair> m_pairs; //TEMP
		
	public:		
		~PhysicsWorld();
		
		float physicsStepTime = 0.0167f;
		vec2 gravity = { 0.0f, 9.81f };
		void update(float inDT);	// converts update into fixed updates
		void simulate(float inDT);	// simulates one time step of physics, call directly if not using update

		cActor* AddActor(const std::vector<vec2>& vertices);
		void RemoveActor(cActor* inActor);
		std::vector<cActor*>& getWorldActors() { return m_actors; }
	};
}