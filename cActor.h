#pragma once

#include "chioriMath.h"
#include "flag.h"
#include "integrators.h"

namespace chiori
{
	enum cActorType
	{
		STATIC = 0,
		DYNAMIC = 1,
		KINEMATIC = 2,
		_COUNT = 3
	};

	struct ActorConfig
	{
		cActorType type = DYNAMIC;	
		vec2 position{ vec2::zero };
		float rotation{ 0.0f };

		vec2 linearVelocity{ vec2::zero };
		float angularVelocity{ 0.0f };
		float linearDamping{ 0.0f };
		float angularDamping{ 0.0f };

		float gravityScale{ 1.0f };
	};

	class cActor
	{
	public:
		cActorType type = DYNAMIC;

		enum // Actor flags
		{
			USE_GRAVITY = (1 << 0),
			IS_DIRTY = (1 << 1)
		};

		int shapeIndex = -1;
		Flag_8 _flags = USE_GRAVITY | IS_DIRTY; // actor setting flags

		vec2 origin{ vec2::zero }; // the body origin (not center of mass)
		vec2 position{ vec2::zero }; // center of mass position in world space
		vec2 deltaPosition{ vec2::zero }; // delta position for the whole time step
		vec2 deltaPosition0{ vec2::zero }; // delta position at the beginning of each sub-step (for sub-stepping)
		vec2 localCenter{ vec2::zero }; // location of center of mass relative to the body origin (local space)
		float rotation{ 0.0f };
		float rotation0{ 0.0f };

		vec2 forces{ vec2::zero };
		float torques{ 0.0f };

		float mass{ 1.0f };
		float invMass{ 1.0f };
		float inertia{ 1.0f };
		float invInertia{ 1.0f };

		vec2 linearVelocity{ vec2::zero };
		float angularVelocity{ 0.0f };

		vec2 linearVelocity0{ vec2::zero };
		float angularVelocity0{ 0.0f };

		float linearDamping{ 0.0f };
		float angularDamping{ 0.0f };

		float gravityScale{ 1.0f };

		int contactList{ -1 };		// the head of the dll (-1 means an invalid head) 
		int contactCount{ 0 };		// the number of contacts

		cActor() {};

		#pragma region Get/Setters
		cTransform getTransform() const
		{
			return { origin, rotation };
		}
		void setTransform(const cTransform& inTfm)
		{
			origin = inTfm.pos;
			rotation = inTfm.rot;
			_flags.set(IS_DIRTY);
		}

		float getMass() const { return mass; }
		void setMass(float inMass)
		{
			if (!fltcmp(mass, inMass))
				_flags.set(IS_DIRTY);
			mass = inMass;
			invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
		}

		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags);
		void setFlags(int inFlags);

		int GetShapeIndex() { return shapeIndex; }
		#pragma endregion
		
		void addForce(const vec2& inForce);
		void addTorque(float inTorque);

		bool operator==(const cActor& inRHS) const {
			return this == &inRHS;
		}


	};


}