#include "pch.h"
#include "physicsWorld.h"
#include "gjk.h"
#include "cprocessing.h" //!!TO REMOVE!!

namespace chiori
{
	void PhysicsWorld::update(float inDT)
	{
		// Accumulate the time since the last frame
		accumulator += inDT;
		
		while (accumulator >= physicsStepTime) {
			simulate(physicsStepTime); // Run your fixed-time physics updates
			accumulator -= physicsStepTime;
		}
	}

	void HandleDegenerateFace(GJKresult& result, cActor& inP, cActor& inT)
	{
		if (result.c1[0] == vec2::zero && result.c1[1] == vec2::zero &&
			result.c2[0] == vec2::zero && result.c2[1] == vec2::zero)
			return;

		auto findClosestFace = [&](const cActor& a, const vec2& n, vec2(&c)[2])
			{
				float maxDot = -FLT_MAX;
				std::vector<vec2> v = a.getVertices();
				size_t size = v.size();
				for (size_t i = 0; i < size; i++) {
					const vec2& p1 = v[i];
					const vec2& p2 = v[(i + 1) % size];

					// Compute the edge vector and its normal
					vec2 edge = p2 - p1;
					vec2 edgeNormal = { edge.y, -edge.x };
					edgeNormal = edgeNormal.normalized();

					float dot = edgeNormal.dot(n);
					if (dot > maxDot) {
						maxDot = dot;
						c[0] = p1;
						c[1] = p2;
					}
				}
			};

		if (result.c1[0] == result.c1[1])
		{
			findClosestFace(inP, result.normal, result.c1);
		}
		
		if (result.c2[0] == result.c2[1])
		{
			findClosestFace(inT, -result.normal, result.c2);
		}
	}

	void PhysicsWorld::simulate(float inDT)
	{
		for (cActor*& a : m_actors)
		{
			if (a->getFlags().isSet(cActor::USE_GRAVITY))
			{
				a->addForce(gravity);
			}
			a->integrate(inDT);
			
			if (!a->_iflags.isSet(cActor::IS_DIRTY))
				continue;
			
			std::vector<vec2> verts = a->getVertices();
			AABB aabb = CreateAABBHull(verts.data(), verts.size());
			m_broadphase.MoveProxy(a->broadphaseID, aabb, a->getVelocity(inDT) * inDT);
			a->_iflags.clear(cActor::IS_DIRTY);
		}

		m_pairs.clear();
		m_broadphase.UpdatePairs(
			[this](void* userDataA, void* userDataB)
			{
				cActor* actorA = static_cast<cActor*>(userDataA);
				cActor* actorB = static_cast<cActor*>(userDataB);
				m_pairs.emplace_back(actorA, actorB);
			}
		);

		for (aPair& pair : m_pairs)
		{
			//int id_a = pair.a->broadphaseID;
			//int id_b = pair.b->broadphaseID;
			//AABB aabbs[2];
			//aabbs[0] = m_broadphase.GetFattenedAABB(id_a);
			//aabbs[1] = m_broadphase.GetFattenedAABB(id_b);

			//CP_Settings_StrokeWeight(2);
			//CP_Settings_Stroke(CP_Color_Create(50, 50, 255, 255));
			//for (int i = 0; i < 2; i++)
			//{
			//	vec2 tl{ aabbs[i].min };
			//	vec2 tr{ aabbs[i].max.x, aabbs[i].min.y };
			//	vec2 br{ aabbs[i].max };
			//	vec2 bl{ aabbs[i].min.x, aabbs[i].max.y };
			//	CP_Graphics_DrawLine(tl.x, tl.y, tr.x, tr.y);
			//	CP_Graphics_DrawLine(tr.x, tr.y, br.x, br.y);
			//	CP_Graphics_DrawLine(br.x, br.y, bl.x, bl.y);
			//	CP_Graphics_DrawLine(bl.x, bl.y, tl.x, tl.y);
			//}
			//CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
		}

		auto drawFunc = [&](const AABB& aabb)
			{
				CP_Settings_StrokeWeight(2);
				CP_Settings_Stroke(CP_Color_Create(50, 50, 255, 255));
				vec2 tl{ aabb.min };
				vec2 tr{ aabb.max.x, aabb.min.y };
				vec2 br{ aabb.max };
				vec2 bl{ aabb.min.x, aabb.max.y };
				CP_Graphics_DrawLine(tl.x, tl.y, tr.x, tr.y);
				CP_Graphics_DrawLine(tr.x, tr.y, br.x, br.y);
				CP_Graphics_DrawLine(br.x, br.y, bl.x, bl.y);
				CP_Graphics_DrawLine(bl.x, bl.y, tl.x, tl.y);
				CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
			};
		
		m_broadphase.GetTree().DisplayTree(drawFunc);

		for (int i = 0; i < m_actors.size(); i++)
		{
			for (int j = i; j < m_actors.size(); j++)
			{
				if (i == j)
					continue;
				
				cActor*& a = m_actors[i];
				cActor*& b = m_actors[j];
				GJKobject gjkA{ a->getPosition(), a->getRotation(), [&](const vec2& inDir) { return a->getSupportPoint(inDir); } };
				GJKobject gjkB{ b->getPosition(), b->getRotation(), [&](const vec2& inDir) { return b->getSupportPoint(inDir); } };

				GJKresult result = CollisionDetection(gjkA, gjkB);
				//HandleDegenerateFace(result, a, b);
				CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
				CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
				CP_Graphics_DrawCircle(result.z1.x, result.z1.y, 8);
				CP_Graphics_DrawCircle(result.z2.x, result.z2.y, 8);
				CP_Settings_StrokeWeight(2);
				CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
				// get normal line
				vec2 normal = result.normal;
				vec2 nxtVert = result.z1 + normal * 100;
				CP_Graphics_DrawLine(result.z1.x, result.z1.y, nxtVert.x, nxtVert.y);
				CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
				//HandleDegenerateFace(result, a, b);
				CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));

				for (const Mvert& w : result.s)
				{
					vec2 a = w.a;
					vec2 b = w.b;
					CP_Graphics_DrawCircle(a.x, a.y, 8);
					CP_Graphics_DrawCircle(b.x, b.y, 8);
				}

				//for (vec2& c : result.c1)
				//{
				//	CP_Graphics_DrawCircle(c.x, c.y, 8);
				//}

				//for (vec2& c : result.c2)
				//{
				//	CP_Graphics_DrawCircle(c.x, c.y, 8);
				//}
				CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
			}
		}
	}

	cActor* PhysicsWorld::AddActor(const std::vector<vec2>& vertices)
	{
		cActor* a = new cActor{ vertices };
		m_actors.emplace_back(a);
		AABB aabb = CreateAABBHull(vertices.data(), vertices.size());
		a->broadphaseID = m_broadphase.CreateProxy(aabb, a);
		return a;
	}

	void PhysicsWorld::RemoveActor(cActor* inActor)
	{
		m_actors.erase(std::remove(m_actors.begin(), m_actors.end(), inActor), m_actors.end());
	}
	
	PhysicsWorld::~PhysicsWorld()
	{
		for (cActor*& a : m_actors)
		{
			delete a;
		}
	}
}