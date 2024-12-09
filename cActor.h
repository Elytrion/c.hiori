#pragma once

#include "chioriMath.h"
#include "flag.h"
#include "integrators.h"

namespace chiori
{
	class PhysicsWorld;
	
	class cActor
	{
	public:
		enum // Actor settings
		{
			SIMULATED = (1 << 0),
			IS_KINEMATIC = (1 << 1),
			USE_GRAVITY = (1 << 2)
		};
	private:	
		friend class PhysicsWorld;
		enum // Internal flags used by the system, should not be touched by the user
		{
			IS_DIRTY = (1 << 0),
			ASLEEP = (1 << 1)
		};
		Flag_8 _iflags = 0;
		int shapeIndex = -1;

		Flag_8 _flags = SIMULATED | USE_GRAVITY;
		vec2 forces{ vec2::zero };
		float torques{ 0.0f };

		cTransform ptfm;

	public:
		cActor() {};
		cActor(const cTransform& inTfm) : tfm{ inTfm } { }
		cTransform tfm;
		float mass, invMass;
		float inertia, invInertia;

		#pragma region Get/Setters
		void setTransform(const cTransform& inTfm)
		{
			if (tfm != inTfm)
				_iflags.set(IS_DIRTY);
			vec2 pdiff = tfm.pos - ptfm.pos;
			ptfm.pos = inTfm.pos - pdiff;
			float rdiff = tfm.rot - ptfm.rot;
			ptfm.rot = inTfm.rot - rdiff;
			tfm = inTfm;	
		}
		const cTransform& getTransform() { return tfm; }
		const float getMass() const { return mass; }
		void setMass(float inMass) { mass = inMass; invMass = 1.0f / inMass; }
		const float getInertia() const { return inertia; }
		void setInertia(float inInertia) { inertia = inInertia; invInertia = 1.0f / inInertia; }

		const vec2 getVelocity(float dt) const;
		void setVelocity(const vec2& inVelocity, float dt);
		const float getAngularVelocity(float dt) const;
		void setAngularVelocity(float inAngularVelocity, float dt);
		
		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags);
		void setFlags(int inFlags);

		int GetShapeIndex() { return shapeIndex; }
		#pragma endregion
		
		void integrate(float dt);
		void addForce(const vec2& inForce);
		void addTorque(float inTorque);

		bool operator==(const cActor& inRHS) const {
			return this == &inRHS;
		}
	};


}