#pragma once

#include "cActor.h"
#include "cShape.h"
#include "broadphase.h"
#include "chioriPool.h"
#include "contact.h"
#include "commons.h"

namespace chiori
{
	class PhysicsWorld
	{
	private:
		using Allocator = DEFAULT_ALLOCATOR;
		float accumulator = 0.0f;
		Broadphase m_broadphase;
		struct ppair // TEMP
		{
			ppair(cShape* ia, cShape* ib) : a{ ia }, b{ ib } {}
			cShape* a;
			cShape* b;
		};
		std::vector<ppair> p_pairs; // TEMP
		
	public:		
		~PhysicsWorld();
		
		float physicsStepTime = 0.0167f;
		vec2 gravity = { 0.0f, 9.81f };
		void update(float inDT);	// converts update into fixed updates
		void simulate(float inDT);	// simulates one time step of physics, call directly if not using update

		cShape* CreateShape(const std::vector<vec2>& vertices);
		cActor* CreateActor(cShape* shape);
		void RemoveShape(cShape* shape);
		void RemoveActor(cActor* inActor);

		void HandleBroadphasePair(void* a, void* b);
		void RunBroadphase();
		void RunNarrowphase();
		void RunSolver();
		
		cPool<cActor, Allocator> p_actors;
		cPool<cShape, Allocator> p_shapes;
		cPool<cContact, Allocator> p_contacts;
	};
}