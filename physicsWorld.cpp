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

	static void computeActorInertia(cPhysicsWorld* world, cActor* actor)
	{
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.
		
		actor->inertia = 0.0f;
		actor->invInertia = 0.0f;
		actor->localCenter = vec2::zero;

		if (actor->type == cActorType::STATIC || actor->type == cActorType::KINEMATIC)
		{
			actor->position = actor->origin;
			return;
		}

		cShape* shape = world->p_shapes[actor->shapeIndex];
		const cPolygon& poly = shape->polygon;
		
		float mass = actor->mass;
		cassert(mass > FLT_EPSILON);
		float area = 0.0f;
		float I = 0.0f;
		int count = poly.count;
		const std::vector<vec2>& vertices = poly.vertices;
		vec2 center = { 0.0f, 0.0f };
		// Get a reference point for forming triangles.
		// Use the first vertex to reduce round-off errors.
		vec2 r = vertices[0];
		const float inv3 = 1.0f / 3.0f;

		for (int32_t i = 1; i < count - 1; ++i)
		{
			// Triangle edges
			vec2 e1 = (vertices[i] - r);
			vec2 e2 = (vertices[i + 1] - r);

			float D = cross(e1, e2);

			float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid, r at origin
			center += (triangleArea * inv3 * (e1 + e2));

			float ex1 = e1.x, ey1 = e1.y;
			float ex2 = e2.x, ey2 = e2.y;

			float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
			float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

			I += (0.25f * inv3 * D) * (intx2 + inty2);
		}
		
		center *= (1.0f / area);
		I *= (mass / area);
		float d = center.sqrMagnitude();
		I += mass * d;
		
		actor->inertia = I;
		actor->invInertia = 1.0f / I;
	}

	int cPhysicsWorld::CreateShape(int inActorIndex, const ShapeConfig& inConfig)
	{
		cShape* n_shape = p_shapes.Alloc();
		int shapeIndex = p_shapes.getIndex(n_shape);
		cActor* actor = p_actors[inActorIndex];

		n_shape->actorIndex = inActorIndex;
		n_shape->setVertices(inConfig.vertices);
		n_shape->friction = inConfig.friction;
		n_shape->restitution = inConfig.restitution;

		cTransform xf = { actor->origin, actor->rotation };
		n_shape->aabb = CreateAABBHull(n_shape->polygon.vertices, xf);
		n_shape->broadphaseIndex = m_broadphase.CreateProxy(n_shape->aabb, n_shape);
		actor->shapeIndex = shapeIndex;

		computeActorInertia(this, actor);

		return shapeIndex;
	}
	
	void cPhysicsWorld::RemoveShape(int inShapeIndex)
	{
		cShape* shape = p_shapes[inShapeIndex];
		m_broadphase.DestroyProxy(shape->broadphaseIndex);
		p_actors[shape->actorIndex]->shapeIndex = -1;
		p_shapes.Free(shape);
	}

	void cPhysicsWorld::RemoveActor(int inActorIndex)
	{
		cActor* inActor = p_actors[inActorIndex];
		RemoveShape(inActor->shapeIndex);
		p_actors.Free(inActor);
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

			cShape* shape = p_shapes[actor->shapeIndex];

			if (actor->_flags.isSet(cActor::IS_DIRTY))
			{
				computeActorInertia(this, actor);
			}

			shape->aabb = CreateAABBHull(shape->polygon.vertices, xf);

			AABB fatAABB = m_broadphase.GetFattenedAABB(shape->broadphaseIndex);

			if (!fatAABB.contains(shape->aabb) || actor->_flags.isSet(cActor::IS_DIRTY)) // moved out of broadphase AABB, significant enough movement to update broadphase
			{
				m_broadphase.MoveProxy(shape->broadphaseIndex, shape->aabb, vec2::zero);
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