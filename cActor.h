#pragma once

#include "vector2.h"
#include "rigidbody.h"
#include "shape.h"

namespace chiori
{
	class cActor
	{
	private:		
		Flag_8 _flags = 0 | (1 << 0) | (1 << 1);


	public:
		enum
		{
			SIMULATION_SHAPE = (1 << 0),
			SCENE_QUERY_SHAPE = (1 << 1),
			TRIGGER_SHAPE = (1 << 2),
			IS_KINEMATIC = (1 << 3),
			IS_STATIC = (1 << 4)
		};
		cActor()
		{
			// We set a default integrator (Explicit Euler Integration)
			integrator = [](vec2& pos, vec2& vel, const vec2& forceSum, float dt) {
				vel += forceSum * dt;
				pos += vel * dt;
				};
		};
		cActor(const std::vector<vec2>& inVertices, const vec2& inPosition = vec2::zero, const vec2& inScale = vec2::one, float inRotation = 0.0f) :
			baseVertices(inVertices),
			position(inPosition),
			scale(inScale),
			rotation(inRotation)
		{
			// We set a default integrator (Explicit Euler Integration)
			integrator = [](vec2& pos, vec2& vel, const vec2& forceSum, float dt) {
				vel += forceSum * dt;
				pos += vel * dt;
				};
		}
		
		std::vector<vec2> baseVertices; // does not have any pos, scale or rotation applied to it
		vec2 position = vec2::zero;
		vec2 prevPosition = vec2::zero;
		vec2 scale = vec2::one;
		float rotation = 0.0f;
		float mass = 1.0f;
		vec2 velocity = vec2::zero;
		float angularVelocity = 0.0f;
		std::function<void(vec2&, vec2&, const vec2&, float)> integrator;

		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags);

		void integrate(const vec2& forceSum, float dt) { integrator(position, velocity, forceSum, dt); }

		bool operator==(const cActor& inRHS) const {
			return this == &inRHS;
		}
	};
}