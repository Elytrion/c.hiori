#include "pch.h"
#include "physicsWorld.h"
#include "gjk.h"
#include "manifold.h"
#include "solver.h"
#include "chioriDebug.h"

namespace chiori
{
	#define MAX_FIXED_UPDATES_PER_FRAME 3 

	int cPhysicsWorld::CreateActor(const ActorConfig& inConfig)
	{
		cActor* n_actor = p_actors.Alloc();

		n_actor->type = inConfig.type;

		if (inConfig.type != cActorType::DYNAMIC)
		{
			n_actor->mass = 0.0f;
			n_actor->invMass = 0.0f;
			n_actor->inertia = 0.0f;
			n_actor->invInertia = 0.0f;
		}

		n_actor->origin = inConfig.position;
		n_actor->position = inConfig.position;

		n_actor->linearVelocity = inConfig.linearVelocity;
		n_actor->angularVelocity = inConfig.angularVelocity;

		n_actor->rot = { inConfig.angle };
		n_actor->linearDamping = inConfig.linearDamping;
		n_actor->angularDamping = inConfig.angularDamping;
		n_actor->gravityScale = inConfig.gravityScale;

		return p_actors.getIndex(n_actor);
	}

	void cPhysicsWorld::RemoveActor(int inActorIndex)
	{
		cActor* actor = p_actors[inActorIndex];
		
		// Destroy the attached contacts
		int edgeKey = actor->contactList;
		while (edgeKey != -1)
		{
			int contactIndex = edgeKey >> 1;
			int edgeList = edgeKey & 1;

			int twinKey = edgeKey ^ 1;
			int twinIndex = twinKey & 1;

			cContact* contact = p_contacts[contactIndex];
			
			cContactEdge* twin = contact->edges + twinIndex;
			
			// Remove contact from other body's doubly linked list
			if (twin->prevKey != -1)
			{
				cContact* prevContact = p_contacts[(twin->prevKey >> 1)];
				cContactEdge* prevEdge = prevContact->edges + (twin->prevKey & 1);
				prevEdge->nextKey = twin->nextKey;
			}

			if (twin->nextKey != -1)
			{
				cContact* nextContact = p_contacts[(twin->nextKey >> 1)];
				cContactEdge* nextEdge = nextContact->edges + (twin->nextKey & 1);
				nextEdge->prevKey = twin->prevKey;
			}
			
			// Check other body's list head
			cActor* other = p_actors[twin->bodyIndex];
			if (other->contactList == twinKey)
			{
				other->contactList = twin->nextKey;
			}

			cassert(other->contactCount > 0);
			other->contactCount -= 1;

			// Remove pair from set
			p_pairs.erase(contact->shapeIndexA, contact->shapeIndexB);

			cContactEdge* edge = contact->edges + edgeList;
			edgeKey = edge->nextKey;
			
			// Free contact
			p_contacts.Free(contact);
		}

		// Delete the attached shapes. This destroys broad-phase proxies.
		int shapeIndex = actor->shapeList;
		while (shapeIndex != -1)
		{
			cShape* shape = p_shapes[shapeIndex];
			shapeIndex = shape->nextShapeIndex;
			
			// The broad-phase proxies only exist if the body does
			m_broadphase.DestroyProxy(shape->broadphaseIndex);

			p_shapes.Free(shape);
		}
		// Free body
		p_actors.Free(actor);
	}

	static void computeActorMass(cPhysicsWorld* w, cActor* b)
	{
		// Compute mass data from shapes. Each shape has its own density.
		b->mass = 0.0f;
		b->invMass = 0.0f;
		b->inertia = 0.0f;
		b->invInertia = 0.0f;
		b->localCenter = cVec2::zero;

		// Static and kinematic bodies have zero mass.
		if (b->type == cActorType::STATIC || b->type == cActorType::KINEMATIC)
		{
			b->position = b->origin;
			return;
		}
		
		// Accumulate mass over all shapes.
		cVec2 localCenter = cVec2::zero;
		int32_t shapeIndex = b->shapeList;
		while (shapeIndex != -1)
		{
			const cShape* s = w->p_shapes[shapeIndex];
			shapeIndex = s->nextShapeIndex;

			if (s->density == 0.0f)
			{
				continue;
			}

			cMassData massData = s->computeMass();

			b->mass += massData.mass;
			localCenter += massData.mass * massData.center;
			b->inertia += massData.I;
		}

		// Compute center of mass.
		if (b->mass > 0.0f)
		{
			b->invMass = 1.0f / b->mass;
			localCenter = (b->invMass * localCenter);
		}

		if (b->inertia > 0.0f)
		{
			// Center the inertia about the center of mass.
			b->inertia -= b->mass * dot(localCenter, localCenter);
			cassert(b->inertia > 0.0f);
			b->invInertia = 1.0f / b->inertia;
		}
		else
		{
			b->inertia = 0.0f;
			b->invInertia = 0.0f;
		}

		// Move center of mass.
		cVec2 oldCenter = b->position;
		b->localCenter = localCenter;
		b->position = b->localCenter.rotated(b->rot) + b->origin;

		// Update center of mass velocity.
		cVec2 deltaLinear = cross(b->angularVelocity, (b->position - oldCenter));
		b->linearVelocity = (b->linearVelocity + deltaLinear);
	}

	int cPhysicsWorld::CreateShape(int inActorIndex, const ShapeConfig& inConfig, cPolygon* inGeom)
	{
		cShape* n_shape = p_shapes.Alloc();
		int shapeIndex = p_shapes.getIndex(n_shape);
		cActor* actor = p_actors[inActorIndex];

		n_shape->actorIndex = inActorIndex;
		n_shape->polygon = *inGeom;
		n_shape->density = inConfig.density;
		n_shape->friction = inConfig.friction;
		n_shape->restitution = inConfig.restitution;

		cTransform xf = actor->getTransform();
		
		n_shape->aabb = n_shape->ComputeAABB(xf);
		n_shape->broadphaseIndex = m_broadphase.CreateProxy(n_shape->aabb, reinterpret_cast<void*>(n_shape->header.index));
		
		// Add to shape linked list
		n_shape->nextShapeIndex = actor->shapeList;
		actor->shapeList = shapeIndex;

		if (n_shape->density)
		{
			computeActorMass(this, actor);
		}

		return shapeIndex;
	}

	void cPhysicsWorld::step(float inFDT, int primaryIterations, int secondaryIterations, bool warmStart)
	{
		int actorCapacity = p_actors.capacity();
		// Step 1: Update the transform and broadphase AABBs for all shapes
		// We also check if any of the actors or shapes have been modified by the user and update the system accordingly
		for (int i = 0; i < actorCapacity; ++i)
		{
			if (!p_actors.isValid(i))
				continue;

			cActor* actor = p_actors[i];
			if (actor->type == cActorType::STATIC)
				continue;

			actor->origin = actor->position - actor->localCenter.rotated(actor->rot);
			actor->forces = cVec2::zero;
			actor->torques = 0.0f;

			cTransform xf = { actor->origin, actor->rot };

			int shapeIndex = actor->shapeList;
			while (shapeIndex != -1)
			{
				cShape* shape = p_shapes[shapeIndex];
				
				shape->aabb = CreateAABBHull(shape->polygon.vertices, shape->polygon.count, xf);
				AABB fatAABB = m_broadphase.GetFattenedAABB(shape->broadphaseIndex);
				if (!fatAABB.contains(shape->aabb) || actor->_flags.isSet(cActor::IS_DIRTY)) // moved out of broadphase AABB, significant enough movement to update broadphase
				{
					m_broadphase.MoveProxy(shape->broadphaseIndex, shape->aabb, cVec2::zero);
				}

				shapeIndex = shape->nextShapeIndex;
			}

			// We might need to recalculate the mass and inertia of the actor their values have been modifed
			if (actor->_flags.isSet(cActor::IS_DIRTY))
			{
				computeActorMass(this, actor);
			}

			if (actor->_flags.isSet(cActor::IS_DIRTY))
				actor->_flags.clear(cActor::IS_DIRTY);
		}

		// Step 2: Broadphase + Narrowphase + Contact Generation
		// Update collision pairs, and create all new contacts for this frame
		// This includes the broadphase AABB tree query, narrowphase using GJK
		// and contact generation in one sweep
		m_broadphase.UpdatePairs(
			[this](void* userDataA, void* userDataB)
			{
				int shapeAIndex = reinterpret_cast<int>(userDataA);
				int shapeBIndex = reinterpret_cast<int>(userDataB);
				if (p_pairs.contains(shapeAIndex, shapeBIndex))
					return; // no need to create a contact for these shapes since a contact already exists
				CreateContact(this, p_shapes[shapeAIndex], p_shapes[shapeBIndex]);
			}
		);

		// Step 3: Update Contacts
		// All contacts are run through and updated or removed
		// as required. We loop backwards to ensure that the
		// removal of items doesn't invalidate the pool loop
		size_t contactCapacity = p_contacts.capacity();
		int i = static_cast<int>(contactCapacity) - 1;
		for (; i >= 0; --i)
		{
			if (!p_contacts.isValid(i))
				continue;

			cContact* contact = p_contacts[i];
			cShape* shapeA = p_shapes[contact->shapeIndexA];
			cShape* shapeB = p_shapes[contact->shapeIndexB];
			AABB aabb_a = m_broadphase.GetFattenedAABB(shapeA->broadphaseIndex);
			AABB aabb_b = m_broadphase.GetFattenedAABB(shapeB->broadphaseIndex);
			bool overlaps = aabb_a.intersects(aabb_b);
			if (overlaps)
			{
				// Shape fat AABBs are still overlapping, so keep this contact
				// and update it with the new info
				cActor* actorA = p_actors[shapeA->actorIndex];
				cActor* actorB = p_actors[shapeB->actorIndex];
				UpdateContact(this, contact, shapeA, actorA, shapeB, actorB);
			}
			else
			{
				DestroyContact(this, contact);
			}
		}

		// Step 4: Integrate velocities, solve velocity constraints,
		// and integrate positions. This is done primary in the solver.
		SolverContext context;
		context.dt = inFDT;
		context.iterations = primaryIterations;
		context.extraIterations = secondaryIterations;
		context.warmStart = warmStart;
		context.inv_dt = (inFDT > 0.0f) ? 1.0f / inFDT : 0.0f;
		context.h = context.dt;
		context.inv_h = context.inv_dt;

		if (runBasicSolver)
		{
			PGSSolver(this, &context);
		}
		else
		{
			PGSSoftSolver(this, &context);
		}
		
	}


	static void DebugDrawShape(cDebugDraw* draw, cShape* shape, cTransform xf, cDebugColor color)
	{
		const cPolygon& poly = shape->polygon;
		int count = poly.count;
		cassert(count <= MAX_POLYGON_VERTICES);
		cVec2 verts[MAX_POLYGON_VERTICES];
		
		for (int i = 0; i < count; ++i)
		{
			verts[i] = cTransformVec(xf, poly.vertices[i]);
		}

		draw->DrawPolygon(verts, count, color, draw->context);
	}

	void cPhysicsWorld::DebugDraw(cDebugDraw* draw)
	{
		float textSize = fontSize;
		if (draw->drawShapes)
		{
			int count = p_actors.capacity();
			for (int i = 0; i < count; ++i)
			{
				if (!p_actors.isValid(i))
					continue;

				cActor* actor = p_actors[i];

				cTransform xf = actor->getTransform();
				int shapeIndex = actor->shapeList;
				while (shapeIndex != NULL_INDEX)
				{
					cShape* shape = p_shapes[shapeIndex];
					if (actor->type == cActorType::DYNAMIC && actor->mass <= 0.0f)
					{
						// Error body!
						DebugDrawShape(draw, shape, xf, cDebugColor::Red);
					}
					else if (actor->type == cActorType::STATIC)
					{
						DebugDrawShape(draw, shape, xf, { 0.5f, 0.9f, 0.5f, 1.0f });
					}
					else if (actor->type == cActorType::KINEMATIC)
					{
						DebugDrawShape(draw, shape, xf, { 0.5f, 0.5f, 0.9f, 1.0f });
					}
					else
					{
						DebugDrawShape(draw, shape, xf, cDebugColor::Yellow);
					}

					shapeIndex = shape->nextShapeIndex;
				}

			}
		}
		
		if (draw->drawAABBs)
		{
			cDebugColor AABBcolor = { 0.9f, 0.3f, 0.9f, 1.0f };

			int count = p_actors.capacity();
			for (int i = 0; i < count; ++i)
			{
				if (!p_actors.isValid(i))
					continue;

				cActor* actor = p_actors[i];

				char buffer[32];
				snprintf(buffer, 32, "%d", i);
				draw->DrawString(actor->position, textSize, buffer, AABBcolor, draw->context);

				int shapeIndex = actor->shapeList;
				while (shapeIndex != NULL_INDEX)
				{
					cShape* shape = p_shapes[shapeIndex];
					AABB aabb = shape->aabb;
					
					cVec2 verts[4] = {
						{aabb.min.x, aabb.min.y},
						{aabb.max.x, aabb.min.y},
						{aabb.max.x, aabb.max.y},
						{aabb.min.x, aabb.max.y}
					};

					draw->DrawPolygon(verts, 4, AABBcolor, draw->context);

					shapeIndex = shape->nextShapeIndex;
				}
			}
		}

		if (draw->drawTreeAABBs)
		{
			cDebugColor treeColor = cDebugColor::Magenta;
			auto drawFunc = [&](int height, const AABB& aabb)
				{
					cVec2 center = aabb.getCenter();
					
					char buffer[32];
					snprintf(buffer, 32, "%d", height);
					draw->DrawString(center, textSize, buffer, treeColor, draw->context);
					
					cVec2 verts[4] = {
						{aabb.min.x, aabb.min.y},
						{aabb.max.x, aabb.min.y},
						{aabb.max.x, aabb.max.y},
						{aabb.min.x, aabb.max.y}
					};

					draw->DrawPolygon(verts, 4, treeColor, draw->context);
				};
			m_broadphase.GetTree().DisplayTree(drawFunc);
		}

		if (draw->drawMass)
		{
			cVec2 offset = { 0.1f,0.1f };
			int count = p_actors.capacity();
			for (int i = 0; i < count; ++i)
			{
				if (!p_actors.isValid(i))
					continue;

				cActor* actor = p_actors[i];
				cTransform xf = actor->getTransform();
				draw->DrawTransform(xf, draw->context);

				cVec2 p = cTransformVec(xf, offset);
				
				char buffer[32];
				snprintf(buffer, 32, "%.2g", actor->mass);
				draw->DrawString(p, textSize, buffer, cDebugColor::White, draw->context);
			}
		}

		if (draw->drawContactPoints)
		{
			const float impulseScale = 1.0f;
			const float axisScale = 0.3f;
			cDebugColor speculativeColor = { 0.3f, 0.3f, 0.3f, 1.0f };
			cDebugColor addColor = { 0.3f, 0.95f, 0.3f, 1.0f };
			cDebugColor persistColor = { 0.1f, 0.1f, 0.95f, 1.0f };
			cDebugColor normalColor = { 0.9f, 0.9f, 0.9f, 1.0f };
			cDebugColor impulseColor = { 0.9f, 0.9f, 0.3f, 1.0f };
			cDebugColor frictionColor = { 0.9f, 0.9f, 0.3f, 1.0f };

			int contactCapacity = p_contacts.capacity();
			for (int i = 0; i < contactCapacity; ++i)
			{
				if (!p_contacts.isValid(i))
					continue;

				cContact* contact = p_contacts[i];
				
				int pointCount = contact->manifold.pointCount;
				cVec2 normal = contact->manifold.normal;
				char buffer[32];

				cActor* actorA = p_actors[contact->edges[0].bodyIndex];
				cTransform xfA = actorA->getTransform();
				
				for (int j = 0; j < pointCount; ++j)
				{
					cManifoldPoint* point = contact->manifold.points + j;
					cVec2 worldPoint = cTransformVec(xfA, point->localAnchorA);

					if (point->separation > commons::LINEAR_SLOP)
					{
						// draw speculative point
						draw->DrawPoint(worldPoint, 5.0f, speculativeColor, draw->context);
					}
					else if (!point->persisted)
					{
						// draw new point
						draw->DrawPoint(worldPoint, 10.0f, addColor, draw->context);
					}
					else if (point->persisted)
					{
						// draw persistent contact point (has existed > 1 frame)
						draw->DrawPoint(worldPoint, 5.0f, persistColor, draw->context);
					}

					if (draw->drawContactImpulses)
					{
						cVec2 p1 = worldPoint;
						cVec2 p2 = p1 + (impulseScale * point->normalImpulse) * normal;
						draw->DrawLine(p1, p2, impulseColor, draw->context);
						snprintf(buffer, 32, "%.2f", point->normalImpulse);
						draw->DrawString(p1, textSize, buffer, impulseColor, draw->context);
					}
					else if (draw->drawContactNormals)
					{
						cVec2 p1 = worldPoint;
						cVec2 p2 = p1 + axisScale * normal;
						draw->DrawLine(p1, p2, normalColor, draw->context);
					}

					if (draw->drawFrictionImpulses)
					{
						cVec2 tangent = { normal.y, -normal.x };
						cVec2 p1 = worldPoint;
						cVec2 p2 = p1 + (impulseScale * point->tangentImpulse) * tangent;
						draw->DrawLine(p1, p2, frictionColor, draw->context);
						snprintf(buffer, 32, "%.2f", point->tangentImpulse);
						draw->DrawString(p1, textSize, buffer, frictionColor, draw->context);
					}
				}
			}
		}
	}

}