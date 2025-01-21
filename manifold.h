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
		cVec2 localAnchorA{ cVec2::zero };
		cVec2 localAnchorB{ cVec2::zero };

		// Friction anchors
		cVec2 frictionAnchorA{ cVec2::zero };
		cVec2 frictionAnchorB{ cVec2::zero };
		cVec2 frictionNormalA{ cVec2::zero };
		cVec2 frictionNormalB{ cVec2::zero };

		float separation{ 0 };
		float normalImpulse{ 0 };
		float tangentImpulse{ 0 };
		uint16_t id;

		bool persisted{ false }; // is this a new or persistant manifold
	};

	struct cManifold
	{
		cManifoldPoint points[2];
		cVec2 normal{ cVec2::zero };
		int pointCount{ 0 };
		int constraintIndex{ -1 };
		bool frictionPersisted{ false };
	};

	cManifold CollideShapes(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB, cGJKCache* cache);
}