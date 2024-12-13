#pragma once
#include "chioriMath.h"

namespace chiori
{
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
		uint16_t id{ 0 };

		bool persisted{ false };
	};

	struct cManifold
	{
		cManifoldPoint points[2];
		vec2 normal{ vec2::zero };
		unsigned pointCount{ 0 };
		unsigned constraintIndex{ 0 };
		bool frictionPersisted{ false };
	};
}