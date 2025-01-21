#pragma once
#include "chioriMath.h" 

namespace chiori
{
	namespace Integrators
	{
		struct IntegratorInput
		{
			cVec2* pos;
			cVec2* prePos;
			cVec2* vel;
			cVec2 forces;
			float* rot;
			float* prevRot;
			float* rotVel;
			float mass;
			float dt;
		};

		inline void ExplicitEuler(IntegratorInput i)
		{
			// Update velocity using the force and mass (Newton's second law)
			*i.vel += (i.forces / i.mass) * i.dt;
			// Update position using the velocity
			*i.pos += *i.vel * i.dt;
			// Update rotation using angular velocity
			*i.rot += (*i.rotVel) * i.dt;
		}

		inline void Verlet(IntegratorInput i)
		{

		}

		inline void ImplictEuler(IntegratorInput i)
		{
			// Update velocity using the force and mass (Newton's second law)
			cVec2 newVel = *i.vel + (i.forces / i.mass) * i.dt;
			// Update position using the new velocity
			*i.pos += newVel * i.dt;
			// Update rotation using angular velocity
			*i.rot += (*i.rotVel) * i.dt;
			// Finally, assign the new velocity back to vel
			*i.vel = newVel;
		}
	}
}