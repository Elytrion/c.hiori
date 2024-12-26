#pragma once
#include "chioriMath.h"

namespace chiori
{
	// forward declaration
	struct cPolygon;
	struct cGJKCache;
	
	struct cManifoldPoint
	{
		// local anchors relative to body origin
		vec2 localAnchorA{ vec2::zero };
		vec2 localAnchorB{ vec2::zero };
		float separation{ 0 };

		// Friction anchors
		vec2 frictionAnchorA;
		vec2 frictionAnchorB;	
		vec2 frictionNormalA;
		vec2 frictionNormalB;

		float normalImpulse;
		float tangentImpulse;

		bool persisted;
	};

	struct cManifold
	{
		cManifoldPoint points[2];
		vec2 normal{ vec2::zero };
		unsigned pointCount{ 0 };
		bool frictionPersisted;
	};

	cManifold CollideShapes(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB, cGJKCache* cache);
}