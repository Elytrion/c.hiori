#pragma once

#include "vector2.h"
#include "flag.h"
#include <cstdint>

namespace chiori
{	
	class Shape 
	{
	private:
		Flag_8 flags = 0 | (1 << 0) | (1 << 1);
		std::vector<vec2> baseVertices; // does not have any pos, scale or rotation applied to it
	public:
		enum
		{
			SIMULATION_SHAPE	= (1 << 0),
			SCENE_QUERY_SHAPE	= (1 << 1),
			TRIGGER_SHAPE		= (1 << 2)
		};	
		
		vec2 localPosition		= vec2::zero;
		vec2 localScale			= vec2::one;
		float localRotation		= 0.0f;

		Shape(vec2 inLocalPosition = vec2::zero, vec2 inLocalScale = vec2::one, float inLocalRotation = 0.0f)
			: localPosition(inLocalPosition), localScale(inLocalScale), localRotation(inLocalRotation) {}

		Flag_8 getFlags() const;
		void setFlags(Flag_8 inFlags);
	};
}