#include "pch.h"
#include "manifold.h"
#include "gjk.h"
#include "cShape.h"

namespace chiori
{
	#define clinearSlop 0.005f
	#define cspeculativeDistance (4.0f * clinearSlop)
	/*
	 * Generates a contact manifold for two convex shapes based on their edges.
	 *
	 * This function computes the contact points between two shapes using the following steps:
	 * 1. Determine the reference and incident edges:
	 *    - The reference edge defines the clipping planes.
	 *    - The incident edge is clipped against the reference edge to determine valid contact points.
	 * 2. Perform scalar clipping:
	 *    - The incident edge's vertices are projected onto the reference edge's tangent.
	 *    - Clipping is performed to identify the overlap of the two edges.
	 *    - Clipped points are computed using linear interpolation.
	 * 3. Calculate separations:
	 *    - The distances of the clipped points from the reference edge along its normal are calculated.
	 * 4. Populate the manifold:
	 *    - The manifold contains up to two contact points, each with:
	 *        - The clipped point relative to the reference shape (localAnchorA).
	 *        - The separation (penetration depth) along the reference normal.
	 *    - The normal direction is flipped if necessary, based on the `flip` flag.
	 *
	 * This method is robust for handling edge-to-edge contact and ensures efficient calculation of
	 * contact points for convex shapes. 
	 */
	cManifold getManifold(const cPolygon* refShape, const cPolygon* incShape, int edgeA, int edgeB)
	{
		cManifold manifold;
		int i11, i12; // reference edge vert indices
		int i21, i22; // incident edge vert indices
		i11 = edgeA;
		i12 = edgeA + 1 < refShape->count ? edgeA + 1 : 0;
		i21 = edgeB;
		i22 = edgeB + 1 < incShape->count ? edgeB + 1 : 0;


		vec2 normal = refShape->normals[i11]; // normal of the reference plane

		const vec2& v11 = refShape->vertices[i11]; // reference edge vertices
		const vec2& v12 = refShape->vertices[i12];
		
		const vec2& v21 = incShape->vertices[i21]; // incident edge vertices
		const vec2& v22 = incShape->vertices[i22];

		vec2 tangent = vec2(-normal.y, normal.x); // Perpendicular to normal

		float lower1 = 0.0f;
		float upper1 = (v12 - v11).dot(tangent);

		// Incident edge points opposite of tangent due to CCW winding
		float upper2 = (v21 - v11).dot(tangent);
		float lower2 = (v22- v11).dot(tangent);
		
		// Compute clipped points
		vec2 vLower, vUpper;
		// Clip lower point
		if (lower2 < lower1 && (upper2 - lower2) > FLT_EPSILON)
		{
			float t = (lower1 - lower2) / (upper2 - lower2);
			vLower = v21 + t * (v22 - v21);
		}
		else
		{
			vLower = v21;
		}
		// Clip upper point
		if (upper2 > upper1 && (upper2 - lower2) > FLT_EPSILON)
		{
			float t = (upper1 - lower2) / (upper2 - lower2);
			vUpper = v21 + t * (v22 - v21);
		}
		else
		{
			vUpper = v22;
		}

		// Compute separations
		float separationLower = (vLower - v11).dot(normal);
		float separationUpper = (vUpper - v11).dot(normal);

		// Populate manifold
		manifold.normal = normal;
		cManifoldPoint& cp1 = manifold.points[0];
		cp1.localAnchorA = vLower;
		cp1.separation = separationLower;
		manifold.pointCount++;
	
		cManifoldPoint& cp2 = manifold.points[1];
		cp2.localAnchorA = vUpper;
		cp2.separation = separationUpper ;
		manifold.pointCount++;

		return manifold;
	}

	cManifold getOverlapManifold(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB, const vec2 normal)
	{
		bool flip = (xfB.pos - xfA.pos).dot(normal) < 0;
		const cPolygon* refShape = (flip) ? shapeB : shapeA;
		const cPolygon* incShape = (flip) ? shapeA : shapeB;
		const cTransform& xfRef = (flip) ? xfB : xfA;
		const cTransform& xfInc = (flip) ? xfA : xfB;
		int edgeRef, edgeInc;
		
		edgeRef = -1;
		float maxAlignment = -FLT_MAX;
		for (int i = 0; i < refShape->count; i++)
		{
			const vec2& ln = refShape->normals[i];
			vec2 wn = ln.rotated(xfRef.rot);
			float alignment = wn.dot(normal);
			if (alignment > maxAlignment)
			{
				maxAlignment = alignment;
				edgeRef = i;
			}
		}

		edgeInc = -1;
		float minAlignment = FLT_MAX;
		for (int i = 0; i < incShape->count; i++)
		{
			const vec2& ln = incShape->normals[i];
			vec2 wn = ln.rotated(xfInc.rot);
			float alignment = wn.dot(-normal);
			if (alignment < minAlignment)
			{
				minAlignment = alignment;
				edgeInc = i;
			}
		}

		return getManifold(refShape, incShape, edgeRef, edgeInc);
	}
	
	cManifold getShapeManifold(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB)
	{
		cManifold manifold = {};
		cPolygon localShapeB;

		cTransform xfRel = InvMulTransforms(xfA, xfB); // we convert shapeB to be in shapeA's local space
		
		localShapeB.count = shapeB->count;
		localShapeB.vertices.resize(localShapeB.count);
		localShapeB.normals.resize(localShapeB.count);
		for (int i = 0; i < localShapeB.count; ++i)
		{
			localShapeB.vertices[i] = cTransformVec(xfRel, shapeB->vertices[i]);
			localShapeB.normals[i] = shapeB->normals[i].rotated(xfRel.rot);
		}
		
		cTransform identity;
		identity.SetIdentity();
		cGJKCache cache = {};
		cache.count = 0;
		cGJKProxy gjka{ shapeA->vertices.data(), shapeA->count };
		cGJKProxy gjkb{ localShapeB.vertices.data(), localShapeB.count };
		cGJKInput input{ gjka, gjkb, identity, identity }; // xfs are identity as we run everything in shapeA local space
		cGJKOutput output;

		cGJK(input, output, &cache);

		if (output.distance > cspeculativeDistance)
		{
			// no contact
			return manifold;
		}

		if (output.distance < 0.1f * clinearSlop)
		{
			cEPA(input, output, &cache);
			manifold = getOverlapManifold(shapeA, &localShapeB, identity, identity, output.normal);
			if (manifold.pointCount > 0)
			{
				manifold.normal.rotate(xfA.rot);
				for (int i = 0; i < manifold.pointCount; ++i)
				{
					manifold.points[i].localAnchorB = cInvTransformVec(xfA, manifold.points[i].localAnchorA);
				}
			}

			return manifold;
		}

		return manifold;
	}
	
}