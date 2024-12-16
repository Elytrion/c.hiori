#pragma once
#include "chioriMath.h"

namespace chiori
{
	class cShape;
	struct cGJKCache;
	struct cManifoldPoint
	{
		// local anchors relative to body origin
		vec2 localAnchorA{ vec2::zero };
		vec2 localAnchorB{ vec2::zero };
		float separation{ 0 };
	};

	struct cManifold
	{
		cManifoldPoint points[2];
		vec2 normal{ vec2::zero };
		unsigned pointCount{ 0 };
	};

	cManifold CollideShapes(const cShape* shapeA, const cTransform& xfA,
		const cShape* shapeB, const cTransform& xfB,
		cGJKCache* cache);
}