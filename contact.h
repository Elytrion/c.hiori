#pragma once
#include "chioriMath.h"
#include "gjk.h"
#include "manifold.h"
#include "flag.h"

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
		enum
		{
			// This contact no longer has overlapping AABBs in the broadphase
			DISJOINT = (1 << 0),
			// This contact started touching after previously being not in contact
			ENTERED = (1 << 1),
			// This contact stopped touching after previously being in contact
			EXITED = (1 << 2),
		};
		Flag_8 flags;
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
