#pragma once

#include "vector2.h"
#include "rigidbody.h"
#include "shape.h"
#include "integrators.h"

namespace chiori
{
	class cActor
	{
	private:		
		Flag_8 _flags = 0 | (1 << 0) | (1 << 1);


	public:
		enum
		{
			SIMULATION_SHAPE = (1 << 0),
			SCENE_QUERY_SHAPE = (1 << 1),
			TRIGGER_SHAPE = (1 << 2),
			IS_KINEMATIC = (1 << 3),
			IS_STATIC = (1 << 4)
		};
		cActor()
		{
			// We set a default integrator (Verlet Integration)
			integrator = Integrators::Verlet;
		};
		cActor(const std::vector<vec2>& inVertices, const vec2& inPosition = vec2::zero, const vec2& inScale = vec2::one, float inRotation = 0.0f) :
			baseVertices(inVertices),
			position(inPosition),
			scale(inScale),
			rotation(inRotation)
		{
			// We set a default integrator (Verlet Integration)
			integrator = Integrators::Verlet;
		}
		
		std::vector<vec2> baseVertices; // does not have any pos, scale or rotation applied to it
		vec2 position = vec2::zero;
		vec2 prevPosition = vec2::zero; 
		vec2 scale = vec2::one;
		float rotation = 0.0f;
		float mass = 1.0f;
		vec2 velocity = vec2::zero;
		float angularVelocity = 0.0f;
		std::function<void(vec2&, vec2&, vec2&, const vec2&, float)> integrator; // users can replace with their own if needed

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
				vertex = vertex.rotate(rotation);
				vertex = vertex + position;
			}
			return vertices;
		}
		const vec2& getPosition() const { return position; }
		void setPosition(const vec2& inPosition) { position = inPosition; prevPosition = inPosition; }
		const vec2& getPrevPosition() const { return prevPosition; }
		void setPrevPosition(const vec2& inPrevPosition) { prevPosition = inPrevPosition; }
		const vec2& getScale() const { return scale; }
		void setScale(const vec2& inScale) { scale = inScale; }
		const float getRotation() const { return rotation; }
		void setRotation(float inRotation) { rotation = inRotation; }
		const float getMass() const { return mass; }
		void setMass(float inMass) { mass = inMass; }
		const vec2& getVelocity() const { return velocity; }
		void setVelocity(const vec2& inVelocity) { velocity = inVelocity; }
		#pragma endregion

		Flag_8 getFlags() const { return _flags; }
		void setFlags(Flag_8 inFlags);
		void setFlags(int inFlags);

		void integrate(const vec2& forceSum, float dt) { integrator(position, prevPosition, velocity, forceSum, dt); }

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