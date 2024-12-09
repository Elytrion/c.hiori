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

	const vec2 cActor::getVelocity(float dt) const
	{
		return (tfm.pos - ptfm.pos) / dt;
	}
	void cActor::setVelocity(const vec2& inVelocity, float dt)
	{
		ptfm.pos = tfm.pos - inVelocity * dt;
	}
	const float cActor::getAngularVelocity(float dt) const
	{
		return (tfm.rot - ptfm.rot) / dt;
	}
	void cActor::setAngularVelocity(float inAngularVelocity, float dt)
	{
		ptfm.rot = tfm.rot - inAngularVelocity * dt;
	}

	void cActor::integrate(float dt)
	{
		if (!_flags.isSet(SIMULATED))
			return;
		if (_flags.isSet(IS_KINEMATIC))
		{
			ptfm.pos = tfm.pos;
			ptfm.rot = tfm.rot;
			forces = vec2::zero;
			torques = 0.0f;
			return;
		}

		vec2& position = tfm.pos;
		vec2& prevPosition = ptfm.pos;
		float& rotation = tfm.rot;
		float& prevRotation = ptfm.rot;

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

		_iflags.set(IS_DIRTY_TFM);

		forces = vec2::zero;
		torques = 0.0f;
	}

	void cActor::addForce(const vec2& inForce)
	{
		forces += inForce * invMass;
	}

	void cActor::addTorque(float torque)
	{
		torques += torque * invInertia;
	}
}