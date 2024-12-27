#pragma once

#include "chioriMath.h"
#include "flag.h"
#include "integrators.h"

namespace chiori
{
	class cActor
	{
	public:
		enum // Actor settings
		{
			SIMULATED = (1 << 0),
			IS_KINEMATIC = (1 << 1),
			USE_GRAVITY = (1 << 2)
		};

		enum // Internal flags used by the system, should not be touched by the user
		{
			IS_DIRTY_TFM = (1 << 0),
			IS_DIRTY_DENSITY = (1 << 1),
			ASLEEP = (1 << 2)
		};
		Flag_8 _iflags = IS_DIRTY_DENSITY; // internal flags
		int shapeIndex = -1;

		Flag_8 _flags = SIMULATED | USE_GRAVITY; // actor setting flags
		vec2 forces{ vec2::zero };
		float torques{ 0.0f };
		float mass, invMass;
		float inertia, invInertia;
		vec2 soffset;				// shape offset from local transform, is the center of mass (COM)
		cTransform ptfm;

		int contactList{ -1 };		// the head of the dll (-1 means an invalid head) 
		int contactCount{ 0 };		// the number of contacts

		cActor() {};
		cActor(const cTransform& inTfm) : tfm{ inTfm } { }
		cTransform tfm;

		#pragma region Get/Setters
		void setTransform(const cTransform& inTfm)
		{
			if (tfm != inTfm)
				_iflags.set(IS_DIRTY_TFM);
			vec2 pdiff = tfm.pos - ptfm.pos;
			ptfm.pos = inTfm.pos - pdiff;
			float rdiff = tfm.rot - ptfm.rot;
			ptfm.rot = inTfm.rot - rdiff;
			tfm = inTfm;	
		}
		const cTransform& getTransform() { return tfm; }
		const float getMass() const { return mass; }
		void setMass(float inMass)
		{
			if (mass != inMass )
				_iflags.set(IS_DIRTY_DENSITY);
			mass = inMass; invMass = 1.0f / inMass;
		}
		const float getInertia() const { return inertia; }
		void setInertia(float inInertia) { inertia = inInertia; invInertia = 1.0f / inInertia; }
		const vec2& getCOMoffset() const { return soffset; }
		void setCOMoffset(const vec2& inOffset)
		{
			if (soffset != inOffset)
				_iflags.set(IS_DIRTY_DENSITY);
			soffset = inOffset;
		}

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