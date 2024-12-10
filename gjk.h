#pragma once

#include "simplex.h"

namespace chiori
{
	class GJKobject // abstracts any geometry with a valid support function into something that can be used by the algorithm
	{
	public:
        const std::vector<vec2>& baseVertices;
        const std::vector<vec2>& baseNormals;
        cTransform tfm;
        vec2 velocity = vec2::zero; // optional
        float angularVelocity = 0.0f; // optional
        float t = -1; // used for TOI collision checking

        GJKobject(const std::vector<vec2>& baseVerts, const std::vector<vec2>& baseNorms, const cTransform& inTfm)
            : baseVertices{ baseVerts }, baseNormals{ baseNorms }, tfm{ inTfm } { }

        vec2 getSupportPoint(const vec2& inDir) const;
	};

    struct GJKSweepObject
    {
        vec2 p0, p1; // The current initial position and final position (positions at [alpha0, 1])
        float r0, r1; // The current initial angle and final angle (angle at [alpha0, 1])
        float alpha0;// Fraction of the current time step in the range [0,1]
        
        // beta is a factor in [0,1], where 0 indicates alpha0.
        inline vec2 getPos(float beta) const
        {
            return (1.0f - beta) * p0 + beta * p1;
        }
        // beta is a factor in [0,1], where 0 indicates alpha0.
        inline float getRot(float beta) const
        {
            return (1.0f - beta) * r0 + beta * r1;
        }

        inline void advance(float alpha)
        {
            if (alpha > 1.0f)
                return;
            float beta = (alpha - alpha0) / (1.0f - alpha0);
            p0 = getPos(beta);
            r0 = getRot(beta);
            alpha0 = alpha;
        }

        inline void normalizeAngles()
        {
            float d = (PI * 2.0f) * floorf(r0);
            r0 -= d; r1 -= d;
        }
    };

    enum GJKRESULTTYPE
    {
        VERT_VERT = 1,
        VERT_EDGE = 2,
        EDGE_EDGE = 3
    };

    struct GJKresult
    {
        float distance; // distance between objs (0 if intersecting)
		vec2 z1; // closest point on primary object (witness point on obj 1)
        vec2 z2; // closest point on target object (witness point on obj 2)
        vec2 normal; // normal of the collision
        Simplex s;
        float intersection_distance; // if distance <= 0, this will be the intersection distance
        vec2 c1[2]; // contributing edge/vertex on primary shape
		vec2 c2[2]; // contributing edge/vertex on target shape
        GJKRESULTTYPE type;
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