#pragma once

#include "simplex.h"

namespace chiori
{
	class GJKobject // abstracts any point cloud into something that can be used by the algorithm
	{
	public:
		std::vector<vec2> vertices;
		vec2 position;
        vec2 velocity = vec2::zero; // optional

        GJKobject(const std::vector<vec2>& vertices, const vec2& position)
            : vertices{ vertices }, position{ position } {}

		vec2 getSupportPoint(const vec2& inDir) const;
	};

    struct GJKresult
    {
        float distance; // Zero if in contact or intersecting
        vec2 zA; // closest point on primary object
        vec2 zB; // closest point on target object
    };
    
    inline Mvert GetSupportVertex(const GJKobject& inA, const GJKobject& inB, const vec2& inDir)
    {
        return Mvert{ inA.getSupportPoint(inDir), inB.getSupportPoint(-inDir) };
    }
	
    GJKresult GJKExtended(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex);


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