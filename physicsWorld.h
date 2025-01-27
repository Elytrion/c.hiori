#pragma once

#include "cActor.h"
#include "cShape.h"
#include "broadphase.h"
#include "chioriPool.h"
#include "contact.h"
#include "commons.h"

namespace chiori
{	
	class cDebugDraw; // forward declaration

	class cPhysicsWorld
	{
	public:
		std::unique_ptr<cAllocator> allocator;
		float accumulator = 0.0f;
		cBroadphase m_broadphase;

		template <typename Allocator = cDefaultAllocator>
		explicit cPhysicsWorld(Allocator alloc = Allocator()) :
			allocator { std::make_unique<cAllocatorWrapper<Allocator>>(std::move(alloc)) },
			p_actors{ allocator.get() }, p_shapes{ allocator.get() }, p_contacts{ allocator.get() }
		{}

		~cPhysicsWorld() = default;
		
		float physicsStepTime = 0.0167f;
		cVec2 gravity = { 0.0f, -9.81f };
		void update(float inDT);	// converts update into fixed updates
		void step(float inDT);	// simulates one time step of physics, call directly if not using update

		int CreateActor(const ActorConfig& inConfig);
		int CreateShape(int inActorIndex, const ShapeConfig& inConfig, cPolygon* inGeom);
		void RemoveActor(int inActorIndex);

		float fontSize = 14.0f;
		void DebugDraw(cDebugDraw* draw);

		cPool<cActor> p_actors;
		cPool<cShape> p_shapes;
		cFLUTable p_pairs;
		cPool<cContact> p_contacts;
	};
}