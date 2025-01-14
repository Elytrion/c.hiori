#include "pch.h"
#include "physicsWorld.h"
#include "gjk.h"
#include "cprocessing.h" //!!TO REMOVE!!
#include "manifold.h"
#include "solver.h"

namespace chiori
{
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

		n_actor->rotation = inConfig.rotation;
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
			int edgeIndex = edgeKey & 1;

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

			cContactEdge* edge = contact->edges + edgeIndex;
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
		b->localCenter = vec2::zero;

		// Static and kinematic bodies have zero mass.
		if (b->type == cActorType::STATIC || b->type == cActorType::KINEMATIC)
		{
			b->position = b->origin;
			return;
		}
		
		// Accumulate mass over all shapes.
		vec2 localCenter = vec2::zero;
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
		vec2 oldCenter = b->position;
		b->localCenter = localCenter;
		b->position = b->localCenter.rotated(b->rotation) + b->origin;

		// Update center of mass velocity.
		vec2 deltaLinear = cross(b->angularVelocity, (b->position - oldCenter));
		b->linearVelocity = (b->linearVelocity + deltaLinear);
	}

	int cPhysicsWorld::CreateShape(int inActorIndex, const ShapeConfig& inConfig, cPolygon* inGeom)
	{
		cShape* n_shape = p_shapes.Alloc();
		int shapeIndex = p_shapes.getIndex(n_shape);
		cActor* actor = p_actors[inActorIndex];

		n_shape->actorIndex = inActorIndex;
		n_shape->polygon = *inGeom;
		n_shape->friction = inConfig.friction;
		n_shape->restitution = inConfig.restitution;

		cTransform xf = { actor->origin, actor->rotation };
		n_shape->aabb = n_shape->ComputeAABB(xf);
		n_shape->broadphaseIndex = m_broadphase.CreateProxy(n_shape->aabb, n_shape);
		
		// Add to shape linked list
		n_shape->nextShapeIndex = actor->shapeList;
		actor->shapeList = shapeIndex;

		if (n_shape->density)
		{
			computeActorMass(this, actor);
		}

		return shapeIndex;
	}



	
	void cPhysicsWorld::update(float inDT)
	{
		// Accumulate the time since the last frame
		accumulator += inDT;
		
		while (accumulator >= physicsStepTime) {
			step(physicsStepTime); 
			accumulator -= physicsStepTime;
		}
	}

	void cPhysicsWorld::step(float inDT)
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

			actor->origin = actor->position - actor->localCenter.rotated(actor->rotation);
			actor->forces = vec2::zero;
			actor->torques = 0.0f;

			cTransform xf = { actor->origin, actor->rotation };

			int shapeIndex = actor->shapeList;
			while (shapeIndex != -1)
			{
				cShape* shape = p_shapes[shapeIndex];
				
				shape->aabb = CreateAABBHull(shape->polygon.vertices, shape->polygon.count, xf);
				AABB fatAABB = m_broadphase.GetFattenedAABB(shape->broadphaseIndex);
				if (!fatAABB.contains(shape->aabb) || actor->_flags.isSet(cActor::IS_DIRTY)) // moved out of broadphase AABB, significant enough movement to update broadphase
				{
					m_broadphase.MoveProxy(shape->broadphaseIndex, shape->aabb, vec2::zero);
				}
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
				cShape* shapeA = static_cast<cShape*>(userDataA);
				cShape* shapeB = static_cast<cShape*>(userDataB);
				int shapeAIndex = p_shapes.getIndex(shapeA);
				int shapeBIndex = p_shapes.getIndex(shapeB);
				//std::cout << shapeAIndex << " and " << shapeBIndex << " are a contact pair" << std::endl;
				if (p_pairs.contains(shapeAIndex, shapeBIndex))
					return; // no need to create a contact for these shapes since a contact already exists
				CreateContact(this, shapeA, shapeB);
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
		// TODO: let user modify these values if needed
		SolverContext context;
		context.dt = inDT;
		context.iterations = 4;
		context.extraIterations = 2;
		context.warmStart = true;
		context.inv_dt = (inDT > 0.0f) ? 1.0f / inDT : 0.0f;
		context.h = context.dt;
		context.inv_h = context.inv_dt;

		//PGSSoftSolver(this, &context);
		PGSSolver(this, &context);
		
		//for (int itr = 0; itr < p_actors.size(); itr++)
		//{
		//	cActor* a = p_actors[itr];

		//	if (a->_iflags.isSet(cActor::IS_DIRTY_DENSITY))
		//	{
		//		cShape* s = p_shapes[a->shapeIndex];
		//		a->setInertia(s->inertia(a->mass, a->tfm.scale, a->soffset));
		//		a->_iflags.clear(cActor::IS_DIRTY_DENSITY);
		//	}

		//	if (a->getFlags().isSet(cActor::USE_GRAVITY))
		//	{
		//		a->addForce(gravity);
		//	}
		//	a->integrate(inDT);

		//	if (!a->_iflags.isSet(cActor::IS_DIRTY_TFM))
		//		continue;
		//	
		//	cShape* s = p_shapes[a->shapeIndex];
		//	std::vector<vec2> verts = s->getVertices(a->tfm);
		//	AABB aabb = CreateAABBHull(verts.data(), verts.size());
		//	m_broadphase.MoveProxy(s->broadphaseIndex, aabb, a->getVelocity(inDT) * inDT);
		//	
		//	a->_iflags.clear(cActor::IS_DIRTY_TFM);
		//	
		//	for (int itr2 = 0; itr2 < p_actors.size(); itr2++)
		//	{
		//		cActor* b = p_actors[itr2];

		//		if (a == b)
		//			continue;

		//		cTransform tfm_a = a->getTransform();
		//		cTransform tfm_b = b->getTransform();
		//		const cShape* shp_a = p_shapes[a->shapeIndex];
		//		const cShape* shp_b = p_shapes[b->shapeIndex];
		//		cGJKCache cache;
		//		cache.count = 0;
		//		cManifold manifold = CollideShapes(&shp_a->polygon, &shp_b->polygon, tfm_a, tfm_b, &cache);
		//		
		//		//cGJKCache cache;
		//		//cache.count = 0;
		//		//cGJKProxy gjka{ shp_a->polygon.vertices.data(), shp_a->polygon.count };
		//		//cGJKProxy gjkb{ shp_b->polygon.vertices.data(), shp_b->polygon.count };
		//		//cGJKOutput result;
		//		//cGJKInput input{ gjka, gjkb, tfm_a , tfm_b };
		//		//cGJK(input, result, &cache);
		//		//if (result.distance < EPSILON && cache.count > 1)
		//		//{
		//		//	cEPA(input, result, &cache);
		//		//}
		//		
		//		if (manifold.normal != vec2::zero)
		//		{
		//			CP_Settings_StrokeWeight(2);
		//			CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
		//			vec2 lineEnd = tfm_a.pos + manifold.normal * 25;
		//			CP_Graphics_DrawLine(tfm_a.pos.x, tfm_a.pos.y, lineEnd.x, lineEnd.y);
		//		}
		//		
		//		vec2 pointA, pointB;
		//		//pointA = manifold.pointA; // cTransformVec(tfm_a, m.points[0].localAnchorA);
		//		//pointB = manifold.pointB; //cTransformVec(tfm_a, m.points[1].localAnchorA);
		//		//CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
		//		//CP_Graphics_DrawCircle(pointA.x, pointA.y, 8);
		//		//CP_Graphics_DrawCircle(pointB.x, pointB.y, 8);
		//		if (manifold.pointCount > 0)
		//		{
		//			// Transform contact points to world space
		//			pointA = cTransformVec(tfm_a, manifold.points[0].localAnchorA);
		//			pointA *= 100;
		//			pointA += middle;
		//			CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
		//			CP_Graphics_DrawCircle(pointA.x, pointA.y, 8); // Draw the first contact point
		//		}
		//		if (manifold.pointCount > 1)
		//		{
		//			pointB = cTransformVec(tfm_a, manifold.points[1].localAnchorA);
		//			pointB *= 100;
		//			pointB += middle;
		//			CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
		//			CP_Graphics_DrawCircle(pointB.x, pointB.y, 8); // Draw the second contact point
		//		}

		//		CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
		//	}
		//}
		
		//auto drawFunc = [&](int height, const AABB& aabb)
		//	{
		//		CP_Settings_StrokeWeight(2);
		//		CP_Settings_Stroke(CP_Color_Create(50, 50, 255, 255));
		//		vec2 aabbv[4];
		//		aabbv[0] = { aabb.min };
		//		aabbv[1] = { aabb.max.x, aabb.min.y };
		//		aabbv[2] = { aabb.max };
		//		aabbv[3] = { aabb.min.x, aabb.max.y };
		//		for (int i = 0; i < 4; i++)
		//		{
		//			int j = (i + 1) % 4;
		//			vec2 p = aabbv[i];
		//			vec2 q = aabbv[j];
		//			p *= 100;
		//			q *= 100;
		//			p += {800, 450};
		//			q += {800, 450};
		//			CP_Graphics_DrawLine(p.x, p.y, q.x, q.y);
		//		}
		//		CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
		//	};
		//m_broadphase.GetTree().DisplayTree(drawFunc);
	}

}