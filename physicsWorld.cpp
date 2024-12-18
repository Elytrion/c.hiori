#include "pch.h"
#include "physicsWorld.h"
#include "gjk.h"
#include "cprocessing.h" //!!TO REMOVE!!
#include "manifold.h"

namespace chiori
{
	cShape* PhysicsWorld::CreateShape(const std::vector<vec2>& vertices)
	{
		cShape* n_shape = p_shapes.Alloc();
		std::vector<vec2> v = n_shape->vertices;
		float index = n_shape->broadphaseIndex;
		n_shape->setVertices(vertices);
		AABB& aabb = n_shape->aabb;
		n_shape->broadphaseIndex = m_broadphase.CreateProxy(n_shape->aabb, n_shape);
		return n_shape;
	}
	
	cActor* PhysicsWorld::CreateActor(cShape* shape)
	{
		cActor* n_actor = p_actors.Alloc();
		n_actor->shapeIndex = p_shapes.getIndex(shape);
		shape->actorIndex = p_actors.getIndex(n_actor);
		return n_actor;
	}
	
	void PhysicsWorld::RemoveShape(cShape* shape)
	{
		m_broadphase.DestroyProxy(shape->broadphaseIndex);
		p_actors[shape->actorIndex]->shapeIndex = -1;
		p_shapes.Free(shape);
	}

	void PhysicsWorld::RemoveActor(cActor* inActor)
	{
		RemoveShape(p_shapes[inActor->shapeIndex]);
		p_actors.Free(inActor);
	}



	
	void PhysicsWorld::update(float inDT)
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

	void PhysicsWorld::simulate(float inDT)
	{
		p_pairs.clear();
		m_broadphase.UpdatePairs(
			[this](void* userDataA, void* userDataB)
			{
				cShape* dataA = static_cast<cShape*>(userDataA);
				cShape* dataB = static_cast<cShape*>(userDataB);
				p_pairs.emplace_back(dataA, dataB);
			}
		);

		for (ppair& pair : p_pairs)
		{
			//cTransform tfm_a = p_actors[pair.a->actorIndex]->getTransform();
			//cTransform tfm_b = p_actors[pair.b->actorIndex]->getTransform();
			//GJKobject a{ pair.a->vertices, pair.a->normals, tfm_a };
			//GJKobject b{ pair.b->vertices, pair.b->normals, tfm_b };
			//GJKresult result = CollisionDetection(a, b);
		}
		
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
				cGJKProxy gjka{ shp_a->vertices.data(), shp_a->count };
				cGJKProxy gjkb{ shp_b->vertices.data(), shp_b->count };
				cGJKOutput result;
				cGJKInput input{ gjka, gjkb, tfm_a , tfm_b };
				cGJK(input, result, &cache);
				if (result.distance < EPSILON && cache.count > 1)
				{
					cEPA(input, result, &cache);
				}
				std::cout << result.distance << std::endl;
				////std::cout << result.distance << std::endl;
				//cManifold m = CollideShapes(shp_a, tfm_a, shp_b, tfm_b, &cache);
				if (result.normal != vec2::zero)
				{
					CP_Settings_StrokeWeight(2);
					CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
					vec2 lineEnd = tfm_a.pos + result.normal * 25;
					CP_Graphics_DrawLine(tfm_a.pos.x, tfm_a.pos.y, lineEnd.x, lineEnd.y);
				}
				
				vec2 pointA, pointB;
				pointA = result.pointA; // cTransformVec(tfm_a, m.points[0].localAnchorA);
				pointB = result.pointB; //cTransformVec(tfm_a, m.points[1].localAnchorA);
				CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
				CP_Graphics_DrawCircle(pointA.x, pointA.y, 8);
				CP_Graphics_DrawCircle(pointB.x, pointB.y, 8);

				

				CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
			}
		}

		//auto drawFunc = [&](int height, const AABB& aabb)
		//	{
		//		CP_Settings_StrokeWeight(2);
		//		CP_Settings_Stroke(CP_Color_Create(50, 50, 255, 255));
		//		vec2 tl{ aabb.min };
		//		vec2 tr{ aabb.max.x, aabb.min.y };
		//		vec2 br{ aabb.max };
		//		vec2 bl{ aabb.min.x, aabb.max.y };
		//		CP_Graphics_DrawLine(tl.x, tl.y, tr.x, tr.y);
		//		CP_Graphics_DrawLine(tr.x, tr.y, br.x, br.y);
		//		CP_Graphics_DrawLine(br.x, br.y, bl.x, bl.y);
		//		CP_Graphics_DrawLine(bl.x, bl.y, tl.x, tl.y);
		//		CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
		//	};
		//
		//m_broadphase.GetTree().DisplayTree(drawFunc);

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
	
	PhysicsWorld::~PhysicsWorld()
	{
		
	}
}