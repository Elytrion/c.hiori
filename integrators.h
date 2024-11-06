#pragma once
#include "vector2.h" 

namespace chiori
{
	namespace Integrators
	{
		inline void ExplicitEuler(vec2& pos, vec2& prePos, vec2& vel, const vec2& forces, float dt)
		{
			(void)prePos;
			pos += vel * dt;
			vel += forces * dt;
		}

		inline void Verlet(vec2& pos, vec2& prePos, vec2& vel, const vec2& forces, float dt)
		{
			vec2 newPos = pos * 2 - prePos + forces * dt * dt;
			prePos = pos;
			pos = newPos;
			vel = pos - prePos;
		}

		inline void ImplictEuler(vec2& pos, vec2& prePos, vec2& vel, const vec2& forces, float dt)
		{
			(void)prePos;
			vel += forces * dt;
			pos += vel * dt;
		}
	}
}