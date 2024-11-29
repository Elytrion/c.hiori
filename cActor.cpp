#include "pch.h"
#include "cActor.h"

namespace chiori
{
	void cActor::CalculateInverseInertia() // https://stackoverflow.com/questions/31106438/calculate-moment-of-inertia-given-an-arbitrary-convex-2d-polygon
	{
		float area = 0.0f;
		vec2 center = vec2::zero;
		float mmoi = 0.0f;
		int n = baseVertices.size();
		vec2 com = position + comOffset;
		
		for (int i = 0; i < n; i++) // we calculate all the MOI for all the triangles in the convex shape
		{
			const vec2& a = (baseVertices[i] * scale.x) - com;
			const vec2& b = (baseVertices[(i + 1) % n] * scale.y) - com;
			
			// Compute step values
			float area_step = a.cross(b) / 2.0f;
			vec2 center_step = (a + b) / 3.0f;
			float mmoi_step =
				area_step * (a.dot(a) + b.dot(b) + a.dot(b)) / 6.0f;
			
			// Accumulate values
			center = (center * area + center_step * area_step) / (area + area_step);
			area += area_step;
			mmoi += mmoi_step;
		}
		// Density is calculated from mass and total area
		float density = mass / area;
		// Scale mmoi by density and adjust to the center of mass
		mmoi *= density;
		mmoi -= mass * center.dot(center); // Parallel axis theorem adjustment
		invInertia = 1.0f / mmoi;
		std::cout << "Inv Inertia: " << invInertia << std::endl;
	}
	
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
		return (position - prevPosition) / dt;
	}
	void cActor::setVelocity(const vec2& inVelocity, float dt)
	{
		prevPosition = position - inVelocity * dt;
	}
	const float cActor::getAngularVelocity(float dt) const
	{
		return (rotation - prevRotation) / dt;
	}
	void cActor::setAngularVelocity(float inAngularVelocity, float dt)
	{
		prevRotation = rotation - inAngularVelocity * dt;
	}

	void cActor::integrate(float dt)
	{
		if (!_flags.isSet(SIMULATED))
			return;
		if (_flags.isSet(IS_STATIC) ||
			_flags.isSet(IS_KINEMATIC))
		{
			prevPosition = position;
			prevRotation = prevRotation;
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

		_iflags.set(IS_DIRTY);

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