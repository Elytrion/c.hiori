#pragma once

#include "chioriMath.h"
#include "flag.h"
#include "integrators.h"
#include "chioriPool.h"

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
		cVec2 position{ cVec2::zero };
		float angle{ 0.0f };

		cVec2 linearVelocity{ cVec2::zero };
		float angularVelocity{ 0.0f };
		float linearDamping{ 0.0f };
		float angularDamping{ 0.0f };

		float gravityScale{ 1.0f };
	};

	class cActor
	{
	public:
		cObjHeader header; // required for pool allocator
		cActorType type = DYNAMIC;

		enum // Actor flags
		{
			USE_GRAVITY = (1 << 0),
			IS_DIRTY = (1 << 1)
		};

		Flag_8 _flags = USE_GRAVITY | IS_DIRTY; // actor setting flags

		cVec2 origin{ cVec2::zero };		// the body origin (not center of mass)
		cVec2 position{ cVec2::zero };		// center of mass position in world space
		cVec2 deltaPosition{ cVec2::zero }; // delta position for the whole time step
		cVec2 localCenter{ cVec2::zero };	// location of center of mass relative to the body origin (local space)
		cRot rot{ cRot::iden };

		cVec2 forces{ cVec2::zero };
		float torques{ 0.0f };

		float mass{ 1.0f };
		float invMass{ 1.0f };
		float inertia{ 1.0f };
		float invInertia{ 1.0f };

		cVec2 linearVelocity{ cVec2::zero };
		float angularVelocity{ 0.0f };

		float linearDamping{ 0.0f };
		float angularDamping{ 0.0f };

		float gravityScale{ 1.0f };

		int contactList{ -1 };		// the head of the dll (-1 means an invalid head) 
		int contactCount{ 0 };		// the number of contacts

		int shapeList{ -1 };		// the ll of shapes on the actor
		
		void* userData{ nullptr };

		cActor() {};

		#pragma region Get/Setters
		cTransform getTransform() const
		{
			return { origin, rot };
		}
		void setTransform(const cTransform& inTfm)
		{
			origin = inTfm.p;
			position = origin;
			rot = inTfm.q;
			_flags.set(IS_DIRTY);
		}

		float getMass() const { return mass; }
		void setMass(float inMass)
		{
			if (type == cActorType::STATIC || type == cActorType::KINEMATIC)
			{
				mass = 0.0f;
				invMass = 0.0f;
				return;
			}

			if (!fltsame(mass, inMass))
				_flags.set(IS_DIRTY);
			mass = inMass;
			invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
		}

		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags)
		{
			_flags = inFlags;
		}
		void setFlags(int inFlags)
		{
			_flags.set(inFlags);
		}

		#pragma endregion
		
		void addForce(const cVec2& inForce)
		{
			forces += inForce * invMass;
		}

		void addTorque(float torque)
		{
			torques += torque * invInertia;
		}

		void applyImpulse(const cVec2& impulse, const cVec2& contactPoint)
		{
			linearVelocity += impulse * invMass;
			angularVelocity += invInertia * cross(contactPoint - position, impulse);
		}

		bool operator==(const cActor& inRHS) const {
			return this == &inRHS;
		}


	};


}