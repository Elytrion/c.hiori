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

		// Friction anchors
		vec2 frictionAnchorA{ vec2::zero };
		vec2 frictionAnchorB{ vec2::zero };
		vec2 frictionNormalA{ vec2::zero };
		vec2 frictionNormalB{ vec2::zero };

		float separation{ 0 };
		float normalImpulse{ 0 };
		float tangentImpulse{ 0 };
		uint16_t id;

		bool persisted{ false }; // is this a new or persistant manifold
	};

	struct cManifold
	{
		cManifoldPoint points[2];
		vec2 normal{ vec2::zero };
		int pointCount{ 0 };
		int constraintIndex{ -1 };
		bool frictionPersisted{ false };
	};

	cManifold CollideShapes(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB, cGJKCache* cache);
}