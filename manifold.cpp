#include "pch.h"
#include "manifold.h"
#include "gjk.h"
#include "cShape.h"

#include "cprocessing.h" //!!TO REMOVE!!

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

	// Polygon clipper used by GJK and SAT to compute contact points when there are potentially two contact points.
	static cManifold s2ClipPolygons(const cPolygon* polyA, const cPolygon* polyB, int edgeA, int edgeB, bool flip)
	{
		cManifold manifold = {};

		// reference polygon
		const cPolygon* poly1;
		int i11, i12;

		// incident polygon
		const cPolygon* poly2;
		int i21, i22;

		if (flip)
		{
			poly1 = polyB;
			poly2 = polyA;
			i11 = edgeB;
			i12 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
			i21 = edgeA;
			i22 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
		}
		else
		{
			poly1 = polyA;
			poly2 = polyB;
			i11 = edgeA;
			i12 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
			i21 = edgeB;
			i22 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
		}

		vec2 normal = poly1->normals[i11];

		// Reference edge vertices
		vec2 v11 = poly1->vertices[i11];
		vec2 v12 = poly1->vertices[i12];

		// Incident edge vertices
		vec2 v21 = poly2->vertices[i21];
		vec2 v22 = poly2->vertices[i22];

		vec2 tangent = cross(1.0f, normal);

		float lower1 = 0.0f;
		float upper1 = (v12 - v11).dot(tangent);

		// Incident edge points opposite of tangent due to CCW winding
		float upper2 = (v21 - v11).dot(tangent);
		float lower2 = (v22 - v11).dot(tangent);


		vec2 vLower;
		if (lower2 < lower1 && upper2 - lower2 > FLT_EPSILON)
		{
			vLower = vlerp(v22, v21, (lower1 - lower2) / (upper2 - lower2));
		}
		else
		{
			vLower = v22;
		}

		vec2 vUpper;
		if (upper2 > upper1 && upper2 - lower2 > FLT_EPSILON)
		{
			vUpper = vlerp(v22, v21, (upper1 - lower2) / (upper2 - lower2));
		}
		else
		{
			vUpper = v21;
		}

		// TODO_ERIN vLower can be very close to vUpper, reduce to one point?

		float separationLower = (vLower - v11).dot(normal);
		float separationUpper = (vUpper - v11).dot(normal);

		if (flip == false)
		{
			manifold.normal = normal;
			cManifoldPoint* cp = manifold.points + 0;

			cp->localAnchorA = vLower;
			cp->separation = separationLower;
			manifold.pointCount += 1;
			cp += 1;

			cp->localAnchorA = vUpper;
			cp->separation = separationUpper;
			manifold.pointCount += 1;
			
		}
		else
		{
			manifold.normal = -normal;
			cManifoldPoint* cp = manifold.points + 0;

			cp->localAnchorA = vUpper;
			cp->separation = separationUpper;
			manifold.pointCount += 1;
			cp += 1;

			cp->localAnchorA = vLower;
			cp->separation = separationLower;
			manifold.pointCount += 1;
		}

		return manifold;
	}

	// Find the max separation between poly1 and poly2 using edge normals from poly1.
	static float s2FindMaxSeparation(int* edgeIndex, const cPolygon* poly1, const cPolygon* poly2)
	{
		int count1 = poly1->count;
		int count2 = poly2->count;
		const std::vector<vec2>& n1s = poly1->normals;
		const std::vector<vec2>& v1s = poly1->vertices;
		const std::vector<vec2>& v2s = poly2->vertices;

		int bestIndex = 0;
		float maxSeparation = -FLT_MAX;
		for (int i = 0; i < count1; ++i)
		{
			// Get poly1 normal in frame2.
			vec2 n = n1s[i];
			vec2 v1 = v1s[i];

			// Find the deepest point for normal i.
			float si = FLT_MAX;
			for (int j = 0; j < count2; ++j)
			{
				float sij = n.dot(v2s[j] - v1);
				if (sij < si)
				{
					si = sij;
				}
			}

			if (si > maxSeparation)
			{
				maxSeparation = si;
				bestIndex = i;
			}
		}

		*edgeIndex = bestIndex;
		return maxSeparation;
	}

	// This function assumes there is overlap
	static cManifold s2PolygonSAT(const cPolygon* polyA, const cPolygon* polyB)
	{
		int edgeA = 0;
		float separationA = s2FindMaxSeparation(&edgeA, polyA, polyB);

		int edgeB = 0;
		float separationB = s2FindMaxSeparation(&edgeB, polyB, polyA);

		bool flip;

		if (separationB > separationA)
		{
			flip = true;
			vec2 searchDirection = polyB->normals[edgeB];

			// Find the incident edge on polyA
			int count = polyA->count;
			const std::vector<vec2>& normals = polyA->normals;
			edgeA = 0;
			float minDot = FLT_MAX;
			for (int i = 0; i < count; ++i)
			{
				float dot = searchDirection.dot(normals[i]);
				if (dot < minDot)
				{
					minDot = dot;
					edgeA = i;
				}
			}
		}
		else
		{
			flip = false;
			vec2 searchDirection = polyA->normals[edgeA];

			// Find the incident edge on polyB
			int count = polyB->count;
			const std::vector<vec2>& normals = polyB->normals;
			edgeB = 0;
			float minDot = FLT_MAX;
			for (int i = 0; i < count; ++i)
			{
				float dot = searchDirection.dot(normals[i]);
				if (dot < minDot)
				{
					minDot = dot;
					edgeB = i;
				}
			}
		}

		return s2ClipPolygons(polyA, polyB, edgeA, edgeB, flip);
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
			//cEPA(input, output, &cache);
			manifold = s2PolygonSAT(shapeA, &localShapeB);
			//manifold = getOverlapManifold(shapeA, &localShapeB, identity, identity, output.normal);
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