#pragma once
#include "chioriMath.h"
#include "gjk.h"
#include "manifold.h"
#include "flag.h"
#include "chioriPool.h"

namespace chiori
{
	// Forward declarations
	class cShape;
	class cActor;
	class cPhysicsWorld;

	// A contact edge is used to connect bodies and contacts together
	// in a contact graph where each body is a node and each contact
	// is an edge. A contact edge belongs to a doubly linked list
	// maintained in each attached body. Each contact has two contact
	// edges, one for each attached body.
	struct cContactEdge
	{
		int bodyIndex;
		int prevKey;
		int nextKey;
	};

	// The class manages contact between two shapes. A contact exists for each overlapping
	// AABB in the broad-phase. Therefore a contact object may exist that has no contact points.
	struct cContact
	{
		cObjHeader header; // required for pool allocator
		enum
		{
			OVERLAP = (1 << 0), // this contact has been created, and these two objects are being tracked as a pair in the broadphase
			ENTERED = (1 << 1), // this contact has just entered a collision when there previously was none
			EXITED = (1 << 2),  // this contact has just exited a pre-existing collision, but still has overlapping AABBs
			DISJOINT = (1 << 3) // Broadphase has reported these objects have non-overlapping AABBs. This contact is marked for destruction in the current frame, and should not be used
		};
		Flag_8 flags { OVERLAP };
		cContactEdge edges[2];
		int shapeIndexA;
		int shapeIndexB;
		cGJKCache cache;
		cManifold manifold;
		// Mixed friction and restitution
		float friction;
		float restitution;
	};

	void CreateContact(cPhysicsWorld* world, cShape* shapeA, cShape* shapeB);
	void DestroyContact(cPhysicsWorld* world, cContact* contact);
	void UpdateContact(cPhysicsWorld* world, cContact* contact, cShape* shapeA, cActor* bodyA, cShape* shapeB, cActor* bodyB);
}
