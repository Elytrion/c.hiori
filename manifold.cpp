#include "pch.h"
#include "manifold.h"
#include "gjk.h"
#include "cShape.h"

#include "cprocessing.h" //!!TO REMOVE!!

namespace chiori
{
	#define clinearSlop 0.005f
	#define cspeculativeDistance (4.0f * clinearSlop)
	#define MAKE_MPT_ID(A, B) ((uint8_t)(A) << 8 | (uint8_t)(B))
	
	// Polygon clipper used by GJK and SAT to compute contact points when there are potentially two contact points.
	static cManifold PolygonScalarClipper(const cPolygon* polyA, const cPolygon* polyB, int edgeA, int edgeB, bool flip)
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

		vec2 tangent = {-normal.y, normal.x};

		float lower1 = 0.0f;
		float upper1 = (v12 - v11).dot(tangent);

		// Incident edge points opposite of tangent due to CCW winding
		float upper2 = (v21 - v11).dot(tangent);
		float lower2 = (v22-  v11).dot(tangent);

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

		float separationLower = (vLower - v11).dot(normal);
		float separationUpper = (vUpper - v11).dot(normal);
		float r1 = polyA->radius;
		float r2 = polyB->radius;
		vec2 radiiNormalLower = normal * (0.5f * (r1 - r2 - separationLower));
		vec2 radiiNormalUpper = normal * (0.5f * (r1 - r2 - separationUpper));
		// Put contact points at midpoint, accounting for radii
		vLower += radiiNormalLower;
		vUpper += radiiNormalUpper;

		float radius = r1 + r2;

		if (flip == false)
		{
			manifold.normal = normal;
			cManifoldPoint* cp = manifold.points + 0;

			{
				cp->localAnchorA = vLower;
				cp->separation = separationLower - radius;
				manifold.pointCount += 1;
				cp->id = MAKE_MPT_ID(i11, i22);
				cp += 1;
			}
			{
				cp->localAnchorA = vUpper;
				cp->separation = separationUpper - radius;
				cp->id = MAKE_MPT_ID(i12, i21);
				manifold.pointCount += 1;
			}
		}
		else
		{
			manifold.normal = -(normal);
			cManifoldPoint* cp = manifold.points + 0;

			{
				cp->localAnchorA = vUpper;
				cp->separation = separationUpper - radius;
				manifold.pointCount += 1;
				cp->id = MAKE_MPT_ID(i21, i12);
				cp += 1;
			}
			{
				cp->localAnchorA = vLower;
				cp->separation = separationLower - radius;
				cp->id = MAKE_MPT_ID(i22, i11);
				manifold.pointCount += 1;
			}
		}

		return manifold;
	}

	// Find the max separation between poly1 and poly2 using edge normals from poly1.
	static float FindMaxSeparation(int* edgeIndex, const cPolygon* poly1, const cPolygon* poly2)
	{
		int count1 = poly1->count;
		int count2 = poly2->count;
		const vec2* n1s = poly1->normals;
		const vec2* v1s = poly1->vertices;
		const vec2* v2s = poly2->vertices;

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

	// SAT + Polygon clipper to determine contact points for solver
	static cManifold PolygonSATClipper(const cPolygon* polyA, const cPolygon* polyB)
	{
		int edgeA = 0;
		float separationA = FindMaxSeparation(&edgeA, polyA, polyB);

		int edgeB = 0;
		float separationB = FindMaxSeparation(&edgeB, polyB, polyA);

		bool flip;

		if (separationB > separationA)
		{
			flip = true;
			vec2 searchDirection = polyB->normals[edgeB];

			// Find the incident edge on polyA
			int count = polyA->count;
			const vec2* normals = polyA->normals;
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
			const vec2* normals = polyB->normals;
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

		return PolygonScalarClipper(polyA, polyB, edgeA, edgeB, flip);
	}
	
	cManifold CollideShapes(const cPolygon* shapeA, const cPolygon* shapeB, const cTransform& xfA, const cTransform& xfB, cGJKCache* cache)
	{
		cManifold manifold = {};
		cPolygon localShapeB;
		float radius = shapeA->radius + shapeB->radius;

		cTransform xfRel = InvMulTransforms(xfA, xfB); // we convert shapeB to be in shapeA's local space
		
		localShapeB.count = shapeB->count;
		for (int i = 0; i < localShapeB.count; ++i)
		{
			localShapeB.vertices[i] = cTransformVec(xfRel, shapeB->vertices[i]);
			localShapeB.normals[i] = shapeB->normals[i].rotated(xfRel.rot);
		}
		
		cTransform identity;
		identity.SetIdentity();
		cGJKProxy gjka{ shapeA->vertices, shapeA->count };
		cGJKProxy gjkb{ localShapeB.vertices, localShapeB.count };
		cGJKInput input{ gjka, gjkb, identity, identity }; // xfs are identity as we run everything in shapeA local space
		cGJKOutput output;
		
		cGJK(input, output, cache);

		if (output.distance > cspeculativeDistance)
		{
			// no contact
			return manifold;
		}

		if (output.distance < 0.1f * clinearSlop)
		{
			//cEPA(input, output, &cache);
			manifold = PolygonSATClipper(shapeA, &localShapeB);
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

		if (cache->count == 1)
		{
			// vertex-vertex collision
			vec2 pA = output.pointA;
			vec2 pB = output.pointB;

			float distance = output.distance;
			vec2 normal = (pB-pA).normalized();
			vec2 radiiNormal = normal * 0.5f * (shapeA->radius - localShapeB.radius - distance);
			vec2 contactPointA = pB + radiiNormal;

			manifold.normal = normal.rotated(xfA.rot);
			cManifoldPoint* cp = manifold.points + 0;
			cp->localAnchorA = contactPointA;
			cp->localAnchorB = cInvTransformVec(xfRel, contactPointA);
			cp->separation = distance - radius;
			manifold.pointCount = 1;
			return manifold;
		}

		// vertex-edge collision
		cassert(cache->count == 2);
		bool flip;
		int countA = shapeA->count;
		int countB = localShapeB.count;
		int edgeA, edgeB;

		int a1 = cache->indexA[0];
		int a2 = cache->indexA[1];
		int b1 = cache->indexB[0];
		int s2x = cache->indexB[1];

		if (a1 == a2)
		{
			// 1 point on A, expect 2 points on B
			cassert(b1 != s2x);

			// Find reference edge that most aligns with vector between closest points.
			// This works for capsules and polygons
			vec2 axis = output.pointA - output.pointB;
			float dot1 = axis.dot(localShapeB.normals[b1]);
			float dot2 = axis.dot(localShapeB.normals[s2x]);
			edgeB = dot1 > dot2 ? b1 : s2x;

			flip = true;

			// Get the normal of the reference edge in polyA's frame.
			axis = localShapeB.normals[edgeB];

			// Find the incident edge on polyA
			// Limit search to edges adjacent to closest vertex on A
			int edgeA1 = a1;
			int edgeA2 = edgeA1 == 0 ? countA - 1 : edgeA1 - 1;
			dot1 = axis.dot(shapeA->normals[edgeA1]);
			dot2 = axis.dot(shapeA->normals[edgeA2]);
			edgeA = dot1 < dot2 ? edgeA1 : edgeA2;
		}
		else
		{
			// Find reference edge that most aligns with vector between closest points.
			// This works for capsules and polygons
			vec2 axis = (output.pointB - output.pointA);
			float dot1 = axis.dot(shapeA->normals[a1]);
			float dot2 = axis.dot(shapeA->normals[a2]);
			edgeA = dot1 > dot2 ? a1 : a2;

			flip = false;

			// Get the normal of the reference edge in polyB's frame.
			axis = shapeA->normals[edgeA];

			// Find the incident edge on polyB
			// Limit search to edges adjacent to closest vertex
			int edgeB1 = b1;
			int edgeB2 = edgeB1 == 0 ? countB - 1 : edgeB1 - 1;
			dot1 = axis.dot(localShapeB.normals[edgeB1]);
			dot2 = axis.dot(localShapeB.normals[edgeB2]);
			edgeB = dot1 < dot2 ? edgeB1 : edgeB2;
		}

		manifold = PolygonScalarClipper(shapeA, &localShapeB, edgeA, edgeB, flip);
		if (manifold.pointCount > 0)
		{
			manifold.normal.rotate(xfA.rot);
			for (int i = 0; i < manifold.pointCount; ++i)
			{
				manifold.points[i].localAnchorB = cInvTransformVec(xfRel, manifold.points[i].localAnchorA);
			}
		}

		return manifold;
	}
	
}