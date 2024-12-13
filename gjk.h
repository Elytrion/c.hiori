#pragma once

#include "simplex.h"

namespace chiori
{        
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

    /*
    * A proxy object used by the GJK algorithm that
    * Encapsulates any shape
    */
    struct cGJKProxy
    {
        cGJKProxy(const vec2* vertices, int count, float radius = 0.0f) 
            : m_vertices{ vertices }, m_count{ count }, m_radius{ radius } {}

        int getSupport(const vec2& d) const;            // get the index of the support vertex
        const vec2& GetVertex(int index) const;         // get a specified vertex

		const vec2* m_vertices; // the vertices of the shape
        int m_count;            // the number of vertices
        float m_radius;         // the radius of the shape (TODO)
    };

    /*
    * Input for GJK algorithm
    */
    struct cGJKInput
    {
        cGJKProxy proxyA;  // the primary proxy
        cGJKProxy proxyB;  // the target proxy
        cTransform transformA;  // the transform of the primary proxy
        cTransform transformB;  // the transform of the target proxy
        
        int maxIterations = commons::GJK_ITERATIONS;     // the maximum number of iterations GJK is allowed to run
        float tolerance = EPSILON;          // the tolerance used by the algorithm (EPSILON)
        bool useRadii = false;          // Use the proxy radii for calculating GJK for curved shapes (TODO)
    };

    /*
    * Output for GJK algorithm
    */
    struct cGJKOutput
    {
        vec2 pointA;		// closest point on shapeA
        vec2 pointB;		// closest point on shapeB
        float distance;     // distance between two shapes (if distance is roughly <= 0 (within LEPSILON tolerance) it is an overlapping collision)
        int iterations;     // the number of iterations the GJK ran for, used for determining efficiency
    };

    /*
    * Caches the simplex for warm starting
    */
    struct cGJKCache
    {
        float metric;       // the metric used to validate the cache
        unsigned count;     // the size of the simplex
        unsigned indexA[3]; // supporting vertices on shape A (primary)
        unsigned indexB[3]; // supporting on shape B (target)
    };

    inline const vec2& cGJKProxy::GetVertex(int index) const
    {
        cassert(0 <= index && index < m_count);
        return m_vertices[index];
    }
    inline int cGJKProxy::getSupport(const vec2& d) const
    {
        int bestIndex = 0;
        float bestValue = dot(m_vertices[0], d);
        for (int i = 1; i < m_count; ++i)
        {
            float value = dot(m_vertices[i], d);
            if (value > bestValue)
            {
                bestIndex = i;
                bestValue = value;
            }
        }

        return bestIndex;
    }

    void cGJK(const cGJKInput& input, cGJKOutput& output, cGJKCache* cache = nullptr);
}