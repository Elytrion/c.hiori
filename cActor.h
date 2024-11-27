#pragma once

#include "vector2.h"
#include "rigidbody.h"
#include "shape.h"
#include "integrators.h"

namespace chiori
{
	class cActor
	{
	public:
		enum
		{
			SIMULATED = (1 << 0),
			SCENE_QUERYABLE = (1 << 1),
			TRIGGER = (1 << 2),
			IS_KINEMATIC = (1 << 3),
			IS_STATIC = (1 << 4),
			USE_GRAVITY = (1 << 5)
		};
	private:		
		Flag_8 _flags = SIMULATED | SCENE_QUERYABLE | USE_GRAVITY;
		vec2 forces{ vec2::zero };
		float torques{ 0.0f };

	public:
		cActor(){}
		cActor(const std::vector<vec2>& inVertices, const vec2& inPosition = vec2::zero, const vec2& inScale = vec2::one, float inRotation = 0.0f) :
			baseVertices(inVertices),
			position(inPosition),
			scale(inScale),
			rotation(inRotation)
		{}
		
		std::vector<vec2> baseVertices; // does not have any pos, scale or rotation applied to it
		vec2 position{ vec2::zero };
		vec2 prevPosition{ vec2::zero };
		vec2 scale { vec2::one };
		float rotation{ 0.0f };			// stored in radians
		float prevRotation{ 0.0f };		// stored in radians
		float mass{ 1.0f };
		vec2 velocity{ vec2::zero };
		float angularVelocity{ 0.0f };

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
				vertex = vertex.rotate(rotation * commons::RAD2DEG);
				vertex = vertex + position;
			}
			return vertices;
		}
		const vec2& getPosition() const { return position; }
		void setPosition(const vec2& inPosition)
		{ 
			vec2 diff = position - prevPosition;
			position = inPosition;
			prevPosition = position - diff;
		}
		const vec2& getPrevPosition() const { return prevPosition; }
		void setPrevPosition(const vec2& inPrevPosition) { prevPosition = inPrevPosition; }
		const vec2& getScale() const { return scale; }
		void setScale(const vec2& inScale) { scale = inScale; }
		const float getRotation() const { return rotation; }
		void setRotation(float inRotation)
		{
			float diff = rotation - prevRotation;
			rotation = inRotation;
			prevRotation = rotation - diff;
		}
		const float getPrevRotation() const { return prevRotation; }
		void setPrevRotation(float inPrevRotation) { prevRotation = inPrevRotation; }
		const float getMass() const { return mass; }
		void setMass(float inMass) { mass = inMass; }
		const vec2& getVelocity() const { return velocity; }
		void setVelocity(const vec2& inVelocity) { velocity = inVelocity; }
		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags);
		void setFlags(int inFlags);
		#pragma endregion
		void toggleGravity() { _flags.toggle(USE_GRAVITY); }
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