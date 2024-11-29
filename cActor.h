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
			SCENE_QUERYABLE = (1 << 1),
			TRIGGER = (1 << 2),
			IS_KINEMATIC = (1 << 3),
			IS_STATIC = (1 << 4),
			USE_GRAVITY = (1 << 5)
		};
	private:	
		friend class PhysicsWorld;
		
		enum // Internal flags used by the system, should not be touched by the user
		{
			IS_DIRTY = (1 << 0)
		};
		Flag_8 _iflags = 0;
		int broadphaseID = -1;

		Flag_8 _flags = SIMULATED | SCENE_QUERYABLE | USE_GRAVITY;
		vec2 forces{ vec2::zero };
		float torques{ 0.0f };
		float invInertia{ 0.0f };
		float invMass{ 0.0f };
		vec2 comOffset{ vec2::zero }; // center of mass offset from position
		float prevRotation{ 0.0f };		// stored in radians
		vec2 prevPosition{ vec2::zero };
		void CalculateInverseInertia();

	public:
		cActor(){}
		cActor(const std::vector<vec2>& inVertices, const vec2& inPosition = vec2::zero, const vec2& inScale = vec2::one, float inRotation = 0.0f) :
			baseVertices(inVertices),
			position(inPosition),
			scale(inScale),
			rotation(inRotation)
		{
			CalculateInverseInertia();
		}
		
		std::vector<vec2> baseVertices; // does not have any pos, scale or rotation applied to it, use GetVertices() to get the transformed vertices
		vec2 position{ vec2::zero };
		vec2 scale { vec2::one };
		float rotation{ 0.0f };			// stored in radians
		float mass{ 1.0f };

		#pragma region Get/Setters
		const std::vector<vec2>& getBaseVertices() const { return baseVertices; }
		void setBaseVertices(const std::vector<vec2>& inVertices) { baseVertices = inVertices; }
		std::vector<vec2> getVertices() const {
			std::vector<vec2> vertices = baseVertices;
			// Apply position, scale and rotation to base vertices
			for (auto& vertex : vertices)
			{
				vertex.x = vertex.x * scale.x;
				vertex.y = vertex.y * scale.y;
				vertex = vertex.rotate(rotation * RAD2DEG);
				vertex = vertex + position;
			}
			return vertices;
		}
		const vec2& getPosition() const { return position; }
		void setPosition(const vec2& inPosition)
		{ 
			if (position != inPosition)
				_iflags.set(IS_DIRTY);
			vec2 diff = position - prevPosition;
			position = inPosition;
			prevPosition = position - diff;
		}
		const vec2& getScale() const { return scale; }
		void setScale(const vec2& inScale)
		{
			if (scale != inScale)
				_iflags.set(IS_DIRTY);
			scale = inScale;
		}
		const float getRotation() const { return rotation; }
		void setRotation(float inRotation)
		{
			if (rotation != inRotation)
				_iflags.set(IS_DIRTY);
			float diff = rotation - prevRotation;
			rotation = inRotation;
			prevRotation = rotation - diff;
		}
		const float getMass() const { return mass; }
		void setMass(float inMass) { mass = inMass; invMass = 1.0f / inMass; }
		const vec2& getCOMOffset() { return comOffset; }
		void setCOMOffset(const vec2& inOffset) { comOffset = inOffset; CalculateInverseInertia(); }
		float getInvInertia() const { return invInertia; }

		const vec2 getVelocity(float dt) const;
		void setVelocity(const vec2& inVelocity, float dt);
		const float getAngularVelocity(float dt) const;
		void setAngularVelocity(float inAngularVelocity, float dt);
		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags);
		void setFlags(int inFlags);
		#pragma endregion
		
		void integrate(float dt);
		void addForce(const vec2& inForce);
		void addTorque(float inTorque);

		bool operator==(const cActor& inRHS) const {
			return this == &inRHS;
		}

		inline vec2 getSupportPoint(const vec2& inDir)
		{
			vec2 result = baseVertices[0];
			float maxDot = result.dot(inDir);
			for (int i = 1; i < baseVertices.size(); i++)
			{
				vec2 vertex = baseVertices[i];
				float dot = vertex.dot(inDir);
				if (dot > maxDot)
				{
					maxDot = dot;
					result = vertex;
				}
			}
			return result;
		}
	};
}