#pragma once
#include "chioriMath.h"

namespace chiori
{
	// forward declaration
	struct cPolygon;

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

	cManifold getShapeManifold(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB);
}