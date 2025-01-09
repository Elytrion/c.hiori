#include "pch.h"
#include "contact.h"
#include "cShape.h"
#include "physicsWorld.h"

namespace chiori
{
	static constexpr int NULL_INDEX = -1;
	
	void CreateContact(cPhysicsWorld* world, cShape* shapeA, cShape* shapeB)
	{
		cContact* contact = world->p_contacts.Alloc();
		int contactIndex = world->p_contacts.getIndex(contact);
		contact->shapeIndexA = world->p_shapes.getIndex(shapeA);
		contact->shapeIndexB = world->p_shapes.getIndex(shapeB);
		contact->friction = sqrtf(shapeA->friction * shapeB->friction); // friction mixing
		contact->restitution = (shapeA->restitution > shapeB->restitution) ? shapeA->restitution : shapeB->restitution; // restitution mixing
		
		cActor* bodyA = world->p_actors[shapeA->actorIndex];
		cActor* bodyB = world->p_actors[shapeB->actorIndex];

		contact->edges[0].bodyIndex = shapeA->actorIndex;
		contact->edges[0].prevKey = NULL_INDEX;
		contact->edges[0].nextKey = bodyA->contactList;

		// Connect contact to actor A
		int keyA = (contactIndex << 1) | 0;
		if (bodyA->contactList != NULL_INDEX)
		{
			cContact* contactA = world->p_contacts[(bodyA->contactList >> 1)];
			cContactEdge* edgeA = contactA->edges + (bodyA->contactList & 1);
			edgeA->prevKey = keyA;
		}
		bodyA->contactList = keyA;
		bodyA->contactCount += 1;


		// Connect contact to actor B
		contact->edges[1].bodyIndex = shapeB->actorIndex;
		contact->edges[1].prevKey = NULL_INDEX;
		contact->edges[1].nextKey = bodyB->contactList;

		int32_t keyB = (contactIndex << 1) | 1;
		if (bodyB->contactList != NULL_INDEX)
		{
			cContact* contactB = world->p_contacts[(bodyB->contactList >> 1)];
			cContactEdge* edgeB = contactB->edges + (bodyB->contactList & 1);
			edgeB->prevKey = keyB;
		}
		bodyB->contactList = keyB;
		bodyB->contactCount += 1;

		world->p_pairs.insert(contact->shapeIndexA, contact->shapeIndexB);		
	}
	
	void DestroyContact(cPhysicsWorld* world, cContact* contact)
	{
		world->p_pairs.erase(contact->shapeIndexA, contact->shapeIndexB);

		cContactEdge* edgeA = contact->edges + 0;
		cContactEdge* edgeB = contact->edges + 1;

		cActor* bodyA = world->p_actors[edgeA->bodyIndex];
		cActor* bodyB = world->p_actors[edgeB->bodyIndex];

		int contactIndex = world->p_contacts.getIndex(contact);

		// Remove from body A
		if (edgeA->prevKey != NULL_INDEX)
		{
			cContact* prevContact = world->p_contacts[(edgeA->prevKey >> 1)];
			cContactEdge* prevEdge = prevContact->edges + (edgeA->prevKey & 1);
			prevEdge->nextKey = edgeA->nextKey;
		}

		if (edgeA->nextKey != NULL_INDEX)
		{
			cContact* nextContact = world->p_contacts[(edgeA->nextKey >> 1)];
			cContactEdge* nextEdge = nextContact->edges + (edgeA->nextKey & 1);
			nextEdge->prevKey = edgeA->prevKey;
		}

		int edgeKeyA = (contactIndex << 1) | 0;
		if (bodyA->contactList == edgeKeyA)
		{
			bodyA->contactList = edgeA->nextKey;
		}

		bodyA->contactCount -= 1;

		// Remove from body B
		if (edgeB->prevKey != NULL_INDEX)
		{
			cContact* prevContact = world->p_contacts[(edgeB->prevKey >> 1)];
			cContactEdge* prevEdge = prevContact->edges + (edgeB->prevKey & 1);
			prevEdge->nextKey = edgeB->nextKey;
		}

		if (edgeB->nextKey != NULL_INDEX)
		{
			cContact* nextContact = world->p_contacts[(edgeB->nextKey >> 1)];
			cContactEdge* nextEdge = nextContact->edges + (edgeB->nextKey & 1);
			nextEdge->prevKey = edgeB->prevKey;
		}

		int edgeKeyB = (contactIndex << 1) | 1;
		if (bodyB->contactList == edgeKeyB)
		{
			bodyB->contactList = edgeB->nextKey;
		}

		bodyB->contactCount -= 1;

		world->p_contacts.Free(contact); // free the contact for the pool to use
	}

	void UpdateContact(cPhysicsWorld* world, cContact* contact, cShape* shapeA, cActor* bodyA, cShape* shapeB, cActor* bodyB)
	{
		cManifold oldManifold = contact->manifold;

		int shapeAIndex = world->p_shapes.getIndex(shapeA);
		int shapeBIndex = world->p_shapes.getIndex(shapeB);

		cassert(shapeAIndex == contact->shapeIndexA);
		cassert(shapeBIndex == contact->shapeIndexB);

		bool touching = false;
		contact->manifold.pointCount = 0;

		// bool wasTouching = (contact->flags & s2_contactTouchingFlag) == s2_contactTouchingFlag;


		cTransform transformA = bodyA->getTransform();
		cTransform transformB = bodyB->getTransform();
		contact->manifold = CollideShapes(&shapeA->polygon, &shapeB->polygon, transformA, transformB, &contact->cache);

		touching = contact->manifold.pointCount > 0;

		contact->manifold.frictionPersisted = true;

		if (contact->manifold.pointCount != oldManifold.pointCount)
		{
			contact->manifold.frictionPersisted = false;
		}

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int i = 0; i < contact->manifold.pointCount; ++i)
		{
			cManifoldPoint* mp2 = contact->manifold.points + i;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			mp2->persisted = false;
			uint16_t id2 = mp2->id;

			for (int j = 0; j < oldManifold.pointCount; ++j)
			{
				cManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id == id2)
				{
					mp2->frictionNormalA = mp1->frictionNormalA;
					mp2->frictionNormalB = mp1->frictionNormalB;
					mp2->frictionAnchorA = mp1->frictionAnchorA;
					mp2->frictionAnchorB = mp1->frictionAnchorB;

					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					mp2->persisted = true;
					break;
				}
			}

			if (mp2->persisted == false)
			{
				contact->manifold.frictionPersisted = false;
			}
		}
	}
}

