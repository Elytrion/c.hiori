#pragma once

#include "chioriMath.h"

namespace chiori
{        
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
        vec2 pointA;		// closest point on shapeA to shapeB (EPA will return the furthest point on shape A into shape B in the direction of the nomal)
        vec2 pointB;		// closest point on shapeB to shapeA (EPA will return the furthest point on shape B into shape A in the direction of the nomal)
        float distance;     // distance between two shapes (if distance is roughly <= 0 (within LEPSILON tolerance) it is an overlapping collision)
        int iterations;     // the number of iterations the GJK ran for, used for determining efficiency
        vec2 normal;        // normal of the collision, left as 0,0 if no EPA is run
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

    void cGJK(const cGJKInput& input, cGJKOutput& output, cGJKCache* cache);

    void cEPA(const cGJKInput& input, cGJKOutput& output, cGJKCache* cache); // TODO: Very unstable, unsure why
}