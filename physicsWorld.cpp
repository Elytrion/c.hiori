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

	void PhysicsWorld::simulate(float inDT)
	{
		//for (cActor& a : actors)
		//{
		//	if (a.getFlags().isSet(cActor::IS_STATIC))
		//		continue;
		//	a.integrate(gravity, inDT);
		//}

		for (int i = 0; i < actors.size(); i++)
		{
			for (int j = i; j < actors.size(); j++)
			{
				if (i == j)
					continue;
				
				cActor& a = actors[i];
				cActor& b = actors[j];
				
				GJKobject gjkA{ a.getVertices(), a.getPosition() };
				GJKobject gjkB{ b.getVertices(), b.getPosition() };

				Simplex s;
					
				GJKresult result = GJKExtended(gjkA, gjkB, s, CP_Input_KeyTriggered(CP_KEY::KEY_Y));

				if (CP_Input_MouseDown(MOUSE_BUTTON_2))
					std::cout << result.distance << std::endl;

				//!!TO REMOVE!!
				for (const auto& m : s)
				{
					CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
					//CP_Graphics_DrawCircle(m.a.x, m.a.y, 8);
					//CP_Graphics_DrawCircle(m.b.x, m.b.y, 8);
					CP_Settings_Fill(CP_Color_Create(127, 255, 127, 255));
					CP_Graphics_DrawCircle(result.zA.x, result.zA.y, 8);
					CP_Graphics_DrawCircle(result.zB.x, result.zB.y, 8);
					CP_Graphics_DrawLine(result.zA.x, result.zA.y, result.zB.x, result.zB.y);
					CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
				}
				
			}
		}
	}

	cActor& PhysicsWorld::AddActor(const std::vector<vec2>& vertices)
	{
		actors.emplace_back(vertices);
		return actors.back();
	}

	void PhysicsWorld::RemoveActor(cActor& inActor)
	{
		actors.erase(std::remove(actors.begin(), actors.end(), inActor), actors.end());
	}
	
}