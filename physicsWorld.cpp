#include "pch.h"
#include "physicsWorld.h"
#include "gjk.h"
#include "cprocessing.h" //!!TO REMOVE!!
#include "manifold.h"

namespace chiori
{
	cShape* cPhysicsWorld::CreateShape(const std::vector<vec2>& vertices)
	{
		cShape* n_shape = p_shapes.Alloc();
		std::vector<vec2> v = n_shape->polygon.vertices;
		float index = n_shape->broadphaseIndex;
		n_shape->setVertices(vertices);
		AABB& aabb = n_shape->aabb;
		n_shape->broadphaseIndex = m_broadphase.CreateProxy(n_shape->aabb, n_shape);
		return n_shape;
	}
	
	cActor* cPhysicsWorld::CreateActor(cShape* shape, cTransform tfm)
	{
		cActor* n_actor = p_actors.Alloc();
		n_actor->shapeIndex = p_shapes.getIndex(shape);
		shape->actorIndex = p_actors.getIndex(n_actor);
		n_actor->tfm = tfm;
		// update the shape AABB in broadphase to new position
		shape->aabb.shift(tfm.pos);
		m_broadphase.MoveProxy(shape->broadphaseIndex, shape->aabb, vec2::zero);
		return n_actor;
	}
	
	void cPhysicsWorld::RemoveShape(cShape* shape)
	{
		m_broadphase.DestroyProxy(shape->broadphaseIndex);
		p_actors[shape->actorIndex]->shapeIndex = -1;
		p_shapes.Free(shape);
	}

	void cPhysicsWorld::RemoveActor(cActor* inActor)
	{
		RemoveShape(p_shapes[inActor->shapeIndex]);
		p_actors.Free(inActor);
	}
	
	void cPhysicsWorld::update(float inDT)
	{
		// Accumulate the time since the last frame
		accumulator += inDT;
		
		while (accumulator >= physicsStepTime) {
			simulate(physicsStepTime); 
			accumulator -= physicsStepTime;
		}
	}

	//void HandleDegenerateFace(GJKresult& result, const std::vector<vec2>& inP, const std::vector<vec2>& inT)
	//{
	//	if (result.c1[0] == vec2::zero && result.c1[1] == vec2::zero &&
	//		result.c2[0] == vec2::zero && result.c2[1] == vec2::zero)
	//		return;
	//	auto findClosestFace = [&](const std::vector<vec2>& a, const vec2& n, vec2(&c)[2])
	//		{
	//			float maxDot = -FLT_MAX;
	//			size_t size = a.size();
	//			for (size_t i = 0; i < size; i++) {
	//				const vec2& p1 = a[i];
	//				const vec2& p2 = a[(i + 1) % size];
	//				// Compute the edge vector and its normal
	//				vec2 edge = p2 - p1;
	//				vec2 edgeNormal = { edge.y, -edge.x };
	//				edgeNormal = edgeNormal.normalized();
	//				float dot = edgeNormal.dot(n);
	//				if (dot > maxDot) {
	//					maxDot = dot;
	//					c[0] = p1;
	//					c[1] = p2;
	//				}
	//			}
	//		};
	//	if (result.c1[0] == result.c1[1])
	//	{
	//		findClosestFace(inP, result.normal, result.c1);
	//	}
	//	
	//	if (result.c2[0] == result.c2[1])
	//	{
	//		findClosestFace(inT, -result.normal, result.c2);
	//	}
	//}

	int recommendedWidth = 1600;
	int recommendedHeight = 900;
	vec2 middle = vec2{ recommendedWidth / 2.0f, recommendedHeight / 2.0f };

	void cPhysicsWorld::simulate(float inDT)
	{
		// Step 1: Broadphase + Narrowphase + Contact Generation
		// Update collision pairs, and create all new contacts for this frame
		// This includes the broadphase AABB tree query, narrowphase using GJK
		// and contact generation in one sweep
		CreateCollisionContacts();

		// Step 2: Update Contacts
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




		// Step 3: Integrate velocities, solve velocity constraints,
		// and integrate positions. This is all done in the solver.

		//for (ppair& pair : p_pairs)
		//{
		//	//cTransform tfm_a = p_actors[pair.a->actorIndex]->getTransform();
		//	//cTransform tfm_b = p_actors[pair.b->actorIndex]->getTransform();
		//	//GJKobject a{ pair.a->vertices, pair.a->normals, tfm_a };
		//	//GJKobject b{ pair.b->vertices, pair.b->normals, tfm_b };
		//	//GJKresult result = CollisionDetection(a, b);
		//}
		
		for (int itr = 0; itr < p_actors.size(); itr++)
		{
			cActor* a = p_actors[itr];

			if (a->_iflags.isSet(cActor::IS_DIRTY_DENSITY))
			{
				cShape* s = p_shapes[a->shapeIndex];
				a->setInertia(s->inertia(a->mass, a->tfm.scale, a->soffset));
				a->_iflags.clear(cActor::IS_DIRTY_DENSITY);
			}

			if (a->getFlags().isSet(cActor::USE_GRAVITY))
			{
				a->addForce(gravity);
			}
			a->integrate(inDT);

			if (!a->_iflags.isSet(cActor::IS_DIRTY_TFM))
				continue;
			
			cShape* s = p_shapes[a->shapeIndex];
			std::vector<vec2> verts = s->getVertices(a->tfm);
			AABB aabb = CreateAABBHull(verts.data(), verts.size());
			m_broadphase.MoveProxy(s->broadphaseIndex, aabb, a->getVelocity(inDT) * inDT);
			
			a->_iflags.clear(cActor::IS_DIRTY_TFM);
			
			for (int itr2 = 0; itr2 < p_actors.size(); itr2++)
			{
				cActor* b = p_actors[itr2];

				if (a == b)
					continue;

				cTransform tfm_a = a->getTransform();
				cTransform tfm_b = b->getTransform();
				const cShape* shp_a = p_shapes[a->shapeIndex];
				const cShape* shp_b = p_shapes[b->shapeIndex];
				cGJKCache cache;
				cache.count = 0;
				cManifold manifold = CollideShapes(&shp_a->polygon, &shp_b->polygon, tfm_a, tfm_b, &cache);
				
				//cGJKCache cache;
				//cache.count = 0;
				//cGJKProxy gjka{ shp_a->polygon.vertices.data(), shp_a->polygon.count };
				//cGJKProxy gjkb{ shp_b->polygon.vertices.data(), shp_b->polygon.count };
				//cGJKOutput result;
				//cGJKInput input{ gjka, gjkb, tfm_a , tfm_b };
				//cGJK(input, result, &cache);
				//if (result.distance < EPSILON && cache.count > 1)
				//{
				//	cEPA(input, result, &cache);
				//}
				
				if (manifold.normal != vec2::zero)
				{
					CP_Settings_StrokeWeight(2);
					CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
					vec2 lineEnd = tfm_a.pos + manifold.normal * 25;
					CP_Graphics_DrawLine(tfm_a.pos.x, tfm_a.pos.y, lineEnd.x, lineEnd.y);
				}
				
				vec2 pointA, pointB;
				//pointA = manifold.pointA; // cTransformVec(tfm_a, m.points[0].localAnchorA);
				//pointB = manifold.pointB; //cTransformVec(tfm_a, m.points[1].localAnchorA);
				//CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
				//CP_Graphics_DrawCircle(pointA.x, pointA.y, 8);
				//CP_Graphics_DrawCircle(pointB.x, pointB.y, 8);
				if (manifold.pointCount > 0)
				{
					// Transform contact points to world space
					pointA = cTransformVec(tfm_a, manifold.points[0].localAnchorA);
					pointA *= 100;
					pointA += middle;
					CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
					CP_Graphics_DrawCircle(pointA.x, pointA.y, 8); // Draw the first contact point
				}
				if (manifold.pointCount > 1)
				{
					pointB = cTransformVec(tfm_a, manifold.points[1].localAnchorA);
					pointB *= 100;
					pointB += middle;
					CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
					CP_Graphics_DrawCircle(pointB.x, pointB.y, 8); // Draw the second contact point
				}

				

				CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
			}
		}
		
		auto drawFunc = [&](int height, const AABB& aabb)
			{
				CP_Settings_StrokeWeight(2);
				CP_Settings_Stroke(CP_Color_Create(50, 50, 255, 255));
				vec2 aabbv[4];
				aabbv[0] = { aabb.min };
				aabbv[1] = { aabb.max.x, aabb.min.y };
				aabbv[2] = { aabb.max };
				aabbv[3] = { aabb.min.x, aabb.max.y };
				for (int i = 0; i < 4; i++)
				{
					int j = (i + 1) % 4;
					vec2 p = aabbv[i];
					vec2 q = aabbv[j];
					p *= 100;
					q *= 100;
					p += {800, 450};
					q += {800, 450};
					CP_Graphics_DrawLine(p.x, p.y, q.x, q.y);
				}
				CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
			};
		m_broadphase.GetTree().DisplayTree(drawFunc);

		//for (int i = 0; i < m_actors.size(); i++)
		//{
		//	for (int j = i; j < m_actors.size(); j++)
		//	{
		//		if (i == j)
		//			continue;
		//		
		//		cActor*& a = m_actors[i];
		//		cActor*& b = m_actors[j];
		//		GJKobject gjkA{ a->getPosition(), a->getRotation(), [&](const vec2& inDir) { return a->getSupportPoint(inDir); } };
		//		GJKobject gjkB{ b->getPosition(), b->getRotation(), [&](const vec2& inDir) { return b->getSupportPoint(inDir); } };
		//		GJKresult result = CollisionDetection(gjkA, gjkB);
		//		HandleDegenerateFace(result, a, b);
		//		CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
		//		CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
		//		CP_Graphics_DrawCircle(result.z1.x, result.z1.y, 8);
		//		CP_Graphics_DrawCircle(result.z2.x, result.z2.y, 8);
		//		CP_Settings_StrokeWeight(2);
		//		CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
		//		 get normal line
		//		vec2 normal = result.normal;
		//		vec2 nxtVert = result.z1 + normal * 100;
		//		CP_Graphics_DrawLine(result.z1.x, result.z1.y, nxtVert.x, nxtVert.y);
		//		CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
		//		HandleDegenerateFace(result, a, b);
		//		CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
		//		for (const Mvert& w : result.s)
		//		{
		//			vec2 a = w.a;
		//			vec2 b = w.b;
		//			CP_Graphics_DrawCircle(a.x, a.y, 8);
		//			CP_Graphics_DrawCircle(b.x, b.y, 8);
		//		}
		//		for (vec2& c : result.c1)
		//		{
		//			CP_Graphics_DrawCircle(c.x, c.y, 8);
		//		}
		//		for (vec2& c : result.c2)
		//		{
		//			CP_Graphics_DrawCircle(c.x, c.y, 8);
		//		}
		//		CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
		//	}
		//}
	}
	
	cPhysicsWorld::~cPhysicsWorld()
	{
		
	}

	
	void cPhysicsWorld::CreateCollisionContacts()
	{
		m_broadphase.UpdatePairs(
			[this](void* userDataA, void* userDataB)
			{
				cShape* shapeA = static_cast<cShape*>(userDataA);
				cShape* shapeB = static_cast<cShape*>(userDataB);
				int shapeAIndex = p_shapes.getIndex(shapeA);
				int shapeBIndex = p_shapes.getIndex(shapeB);
				if (p_pairs.contains(shapeAIndex, shapeBIndex))
					return; // no need to create a contact for these shapes since a contact already exists
				CreateContact(this, shapeA, shapeB);
			}
		);
	}
}