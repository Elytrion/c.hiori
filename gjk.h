#pragma once

#include "simplex.h"

namespace chiori
{
	class GJKobject
	{
	public:
		std::vector<vec2> vertices;
		vec2 position;
		vec2 velocity;

        GJKobject(std::vector<vec2> vertices, vec2 position = vec2::zero, vec2 velocity = vec2::zero)
			: vertices(vertices), position(position), velocity(velocity) {}

		vec2 getSupportPoint(const vec2& inDir) const;

		vec2 getSupportPoint(const GJKobject& inOther, const vec2& inDir) const;
	};
	
	float GJK(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex);


	/*
	
	float epsilon = 0.0001f; // Precision threshold
    int maxIterations = 20;  // Maximum number of iterations

    float lowerBound = 0.0f;  // Start of the time step
    float upperBound = 1.0f;  // End of the time step
    float toi = upperBound;   // Time of Impact

    for (int i = 0; i < maxIterations; ++i) {
        float midPoint = (lowerBound + upperBound) / 2.0f;
    
        // Move object A by midPoint * velocity
        MoveObjectAlongVelocity(objectA, midPoint * relativeVelocity);
    
        // Check for collision using GJK
        bool collision = PerformGJK(objectA, objectB);
    
        if (collision) {
            toi = midPoint;   // Update TOI if a collision is found
            upperBound = midPoint; // Narrow down search to earlier times
        } else {
            lowerBound = midPoint; // Narrow down search to later times
        }
    
        // Stop if time interval is smaller than epsilon
        if ((upperBound - lowerBound) < epsilon) {
            break;
        }
    }

    return toi;  // Return the estimated Time of Impact
	*/
}