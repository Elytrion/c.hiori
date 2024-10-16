#pragma once

#include "vector2.h"
#include "rigidbody.h"
#include "shape.h"

namespace chiori
{
	class cActor
	{
	private:		
		Flag_8 flags = 0 | (1 << 0) | (1 << 1);
		std::vector<vec2> baseVertices; // does not have any pos, scale or rotation applied to it

	public:
		enum
		{
			SIMULATION_SHAPE = (1 << 0),
			SCENE_QUERY_SHAPE = (1 << 1),
			TRIGGER_SHAPE = (1 << 2),
			IS_KINEMATIC = (1 << 3),
			IS_STATIC = (1 << 4)
		};

		cActor(const std::vector<vec2>& inVertices, const vec2& inPosition = vec2::zero, const vec2& inScale = vec2::one, float inRotation = 0.0f) :
			baseVertices(inVertices),
			position(inPosition),
			scale(inScale),
			rotation(inRotation)
		{}
		
		vec2 position = vec2::zero;
		vec2 scale = vec2::one;
		float rotation = 0.0f;

		vec2 velocity = vec2::zero;
		float angularVelocity = 0.0f;

		Flag_8 getFlags() const;
		void setFlags(Flag_8 inFlags);

		const std::vector<vec2>& getVertices() const { return baseVertices; }
	};
}