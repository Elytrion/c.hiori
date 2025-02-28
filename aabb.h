#pragma once

#include "chioriMath.h"

namespace chiori
{
	struct cAABB
	{
		cVec2 min{ cVec2::zero };
		cVec2 max{ cVec2::zero };
		
		cVec2 getCenter() const
		{
			return 0.5f * (max + min);
		}

		cVec2 getExtents() const
		{
			return 0.5f * (max - min);
		}

		float perimeter() const
		{
			return 2.0f * ((max.x - min.x) + (max.y - min.y));
		}

		void merge(const cAABB& inAABB)
		{
			min = cVec2::vmin(min, inAABB.min);
			max = cVec2::vmax(max, inAABB.max);
		}

		void merge(const cAABB& inAABB1, const cAABB& inAABB2)
		{
			min = cVec2::vmin(inAABB1.min, inAABB2.min);
			max = cVec2::vmax(inAABB1.max, inAABB2.max);
		}

		bool contains(const cVec2& inPoint) const
		{
			return (
				inPoint.x >= min.x &&
				inPoint.x <= max.x &&
				inPoint.y >= min.y &&
				inPoint.y <= max.y);
		}

		bool contains(const cAABB& inAABB) const
		{
			return contains(inAABB.min) && contains(inAABB.max);
		}

		bool intersects(const cAABB& inAABB) const
		{			
			bool xAxisColliding = (max.x > inAABB.min.x && inAABB.max.x > min.x);
			bool yAxisColliding = (max.y > inAABB.min.y && inAABB.max.y > min.y);

			return (xAxisColliding && yAxisColliding);
		}

		void shift(const cVec2& inOffset)
		{
			min += inOffset;
			max += inOffset;
		}
	};

	inline cAABB CreateAABBHull(const cVec2* inVertices, int count, const cTransform& xf)
	{
		cVec2 lower = cTransformVec(xf, inVertices[0]);
		cVec2 upper = lower;

		for (int32_t i = 1; i < count; ++i)
		{
			cVec2 v = cTransformVec(xf, inVertices[i]);
			lower = cVec2::vmin(lower, v);
			upper = cVec2::vmax(upper, v);
		}

		// TODO: for curved shapes
		//cVec2 r = { radius, radius };
		//lower = (lower - r);
		//upper = (upper - r);

		return { lower, upper };
	}

	inline cAABB CreateAABB(float hx, float hy)
	{
		cVec2 halfExtents = { hx / 2, hy / 2 };
		cAABB aabb;
		aabb.min = { -hx, -hy };
		aabb.max = { hx, hy };
		return aabb;
	}
}