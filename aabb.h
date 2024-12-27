#pragma once

#include "chioriMath.h"

namespace chiori
{
	struct AABB
	{
		vec2 min{ vec2::zero };
		vec2 max{ vec2::zero };
		
		vec2 getCenter() const
		{
			return 0.5f * (max + min);
		}

		vec2 getExtents() const
		{
			return 0.5f * (max - min);
		}

		float perimeter() const
		{
			return 2.0f * ((max.x - min.x) + (max.y - min.y));
		}

		void merge(const AABB& inAABB)
		{
			min = vec2::vmin(min, inAABB.min);
			max = vec2::vmax(max, inAABB.max);
		}

		void merge(const AABB& inAABB1, const AABB& inAABB2)
		{
			min = vec2::vmin(inAABB1.min, inAABB2.min);
			max = vec2::vmax(inAABB1.max, inAABB2.max);
		}

		bool contains(const vec2& inPoint) const
		{
			return (
				inPoint.x >= min.x &&
				inPoint.x <= max.x &&
				inPoint.y >= min.y &&
				inPoint.y <= max.y);
		}

		bool contains(const AABB& inAABB) const
		{
			return contains(inAABB.min) && contains(inAABB.max);
		}

		bool intersects(const AABB& inAABB) const
		{			
			bool xAxisColliding = (max.x > inAABB.min.x && inAABB.max.x > min.x);
			bool yAxisColliding = (max.y > inAABB.min.y && inAABB.max.y > min.y);

			return (xAxisColliding && yAxisColliding);
		}

		void shift(const vec2& inOffset)
		{
			min += inOffset;
			max += inOffset;
		}
	};

	inline AABB CreateAABBHull(const vec2* inVertices, int inVertexCount)
	{
		AABB aabb;
		aabb.min = inVertices[0];
		aabb.max = inVertices[0];

		for (int i = 1; i < inVertexCount; i++)
		{
			aabb.min = vec2::vmin(aabb.min, inVertices[i]);
			aabb.max = vec2::vmax(aabb.max, inVertices[i]);
		}

		return aabb;
	}
}