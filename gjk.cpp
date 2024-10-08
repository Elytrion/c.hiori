
#include "pch.h"
#include "gjk.h"

namespace chiori
{
	vec2 GJKobject::getSupportPoint(const vec2& inDir) const
	{
		vec2 result = vertices[0];
		float maxDot = result.dot(inDir);

		for (int i = 1; i < vertices.size(); i++)
		{
			float dot = vertices[i].dot(inDir);
			if (dot > maxDot)
			{
				maxDot = dot;
				result = vertices[i];
			}
		}
		return result;
	}

	vec2 GJKobject::getSupportPoint(const GJKobject& inOther, const vec2& inDir) const
	{
		return getSupportPoint(inDir) - inOther.getSupportPoint(-inDir);
	}

	
	
}