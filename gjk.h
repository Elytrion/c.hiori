#pragma once

#include "simplex.h"

namespace chiori
{
	class GJKobject // abstracts any point cloud into something that can be used by the algorithm
	{
    using SupportFunction = std::function<vec2(const vec2&)>;
	public:
        SupportFunction supportFunc;
		vec2 position;
        vec2 velocity = vec2::zero; // optional

        GJKobject(const vec2& pos, const SupportFunction& supFunc)
            : position{ pos }, supportFunc{ supFunc } { supportFunc = supFunc; }

        inline vec2 getSupportPoint(const vec2& inDir) const { return supportFunc(inDir); }
	};

    struct GJKresult
    {
        float distance; // distance between objs (0 if intersecting)
		vec2 z1; // closest point on primary object (witness point on obj 1)
        vec2 z2; // closest point on target object (witness point on obj 2)
        vec2 normal; // normal of the collision
        float intersection_distance; // if distance <= 0, this will be the intersection distance
        vec2 c1[2]; // contributing edge/vertex on primary shape
		vec2 c2[2]; // contributing edge/vertex on target shape
    };

    struct CollisionConfig
    {
        int max_iterations;
        int tolerance;
        int flags; // 1st bit = get contacts, 2nd bit = get distance
    };

    struct CollisionStatus
    {
        vec2 z1;
        vec2 z2;
        
        int max_iterations;
        float tolerance;
        int flags;  // 1st bit = get contacts, 2nd bit = get distance
        
        int gjk_iterations;
		int epa_iterations;

		Simplex simplex;
    };

    static inline Mvert GetSupportVertex(const GJKobject& inA, const GJKobject& inB, const vec2& inDir)
    {
        return Mvert{ inA.getSupportPoint(inDir), inB.getSupportPoint(-inDir) };
    }
	
    GJKresult GJK(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex);

    GJKresult EPA(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex, GJKresult& inResult);

    GJKresult CollisionDetection(const GJKobject& inPrimary, const GJKobject& inTarget);
    
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