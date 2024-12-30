#pragma once

#include "cActor.h"
#include "cShape.h"
#include "broadphase.h"
#include "chioriPool.h"
#include "contact.h"
#include "commons.h"

namespace chiori
{
	class cPhysicsWorld
	{
	public:
		using Allocator = DEFAULT_ALLOCATOR;
		float accumulator = 0.0f;
		Broadphase m_broadphase;
	
		~cPhysicsWorld();
		
		float physicsStepTime = 0.0167f;
		vec2 gravity = { 0.0f, 9.81f };
		void update(float inDT);	// converts update into fixed updates
		void step(float inDT);	// simulates one time step of physics, call directly if not using update

		int CreateActor(const ActorConfig& inConfig);
		int CreateShape(int inActorIndex, const ShapeConfig& inConfig);
		void RemoveShape(int inShapeIndex);
		void RemoveActor(int inActorIndex);

		cPool<cActor, Allocator> p_actors;
		cPool<cShape, Allocator> p_shapes;
		cFLUTable p_pairs;
		cPool<cContact, Allocator> p_contacts;
	};
}