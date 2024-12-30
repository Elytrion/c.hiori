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

	void cActor::addForce(const vec2& inForce)
	{
		forces += inForce * invMass;
	}

	void cActor::addTorque(float torque)
	{
		torques += torque * invInertia;
	}
}