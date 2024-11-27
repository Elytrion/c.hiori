#include "pch.h"
#include "cActor.h"

namespace chiori
{
	void cActor::setFlags(Flag_8 inFlags)
	{
		_flags = inFlags;
	}
	void cActor::setFlags(int inFlags)
	{
		_flags.set(inFlags);
	}

	void cActor::integrate(float dt)
	{
		if (!_flags.isSet(SIMULATED))
			return;
		if (_flags.isSet(IS_STATIC))
		{
			prevPosition = position;
			prevRotation = prevRotation;
			velocity = vec2::zero;
			angularVelocity = 0.0f;
			return;
		}
		if (_flags.isSet(IS_KINEMATIC))
		{
			forces = vec2::zero;
			torques = 0.0f;
			return;
		}

		// Save the current position to use as the "previous position" for the next step
		vec2 currentPosition = position;

		// Compute the next position directly using preprocessed forces
		position += (position - prevPosition) + forces * (dt * dt);

		// Update previous position for the next frame
		prevPosition = currentPosition;

		// Save the current rotation to use as the "previous rotation" for the next step
		float currentRotation = rotation;

		// Compute the next rotation directly using preprocessed torques
		rotation += (rotation - prevRotation) + torques * (dt * dt);

		// Update previous rotation for the next frame
		prevRotation = currentRotation;

		forces = vec2::zero;
		torques = 0.0f;
	}

	void cActor::addForce(const vec2& inForce)
	{
		forces += inForce / mass;
	}

	void cActor::addTorque(float torque)
	{
		torques += torque / (mass * 1 / 3);
	}
}