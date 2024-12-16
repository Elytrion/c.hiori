#include "pch.h"
#include "manifold.h"
#include "gjk.h"
#include "cShape.h"

namespace chiori
{
	static float findMaxSeparation(int& bestEdgeIndex, const cShape* shapeA, const cShape* shapeB)
	{
		int count1 = shapeA->count;
		int count2 = shapeB->count;
		const std::vector<vec2>& n1s = shapeA->normals;
		const std::vector<vec2>& v1s = shapeA->vertices;
		const std::vector<vec2>& v2s = shapeB->vertices;

		bestEdgeIndex = 0;
		float maxSep = -FLT_MAX;
		
		for (int i = 0; i < count1; ++i)
		{
			vec2 normal = n1s[i];
			vec2 v1 = v1s[i];

			// Find the deepest point for normal i.
			float si = FLT_MAX;
			for (int j = 0; j < count2; ++j)
			{
				float sij = dot(normal, (v2s[j] - v1));
				if (sij < si)
				{
					si = sij;
				}
			}

			if (si > maxSep)
			{
				maxSep = si;
				bestEdgeIndex = i;
			}
		}

		return maxSep;
	}

	static int findIncidentEdgeIndex(const cShape* ishape, const vec2& searchDirection) {
		int edge = 0;
		float minDot = FLT_MAX;
		int count = ishape->count;
		const std::vector<vec2>& normals = ishape->normals;
		for (int i = 0; i < count; ++i) {
			float dot = searchDirection.dot(normals[i]);
			if (dot < minDot) {
				minDot = dot;
				edge = i;
			}
		}
		return edge;
	}
	
	static cManifold clipPolygons(const cShape* shapeA, const cShape* shapeB, int edgeA, int edgeB, bool flip)
	{
		cManifold manifold;
		
		const cShape* refShape;
		int r1, r2;

		const cShape* incShape;
		int i1, i2;


		if (flip)
		{
			refShape = shapeB;
			r1 = edgeB;
			r2 = (edgeB + 1) % shapeB->count;
			
			incShape = shapeA;
			i1 = edgeA;
			i2 = (edgeA + 1) % shapeA->count;
		}
		else
		{
			refShape = shapeA;
			r1 = edgeA;
			r2 = (edgeA + 1) % shapeA->count;

			incShape = shapeB;
			i1 = edgeB;
			i2 = (edgeB + 1) % shapeB->count;
		}

		vec2 normal = refShape->normals[r1];

		// Reference edge vertices
		vec2 v11 = refShape->vertices[r1];
		vec2 v12 = refShape->vertices[r2];

		// Incident edge vertices
		vec2 v21 = incShape->vertices[i1];
		vec2 v22 = incShape->vertices[i2];

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


		float separationLower = (vLower - v11).dot(normal);
		float separationUpper = (vUpper - v11).dot(normal);


		if (flip == false)
		{
			manifold.normal = normal;
			cManifoldPoint* cp = manifold.points + 0;

			{
				cp->localAnchorA = vLower;
				cp->separation = separationLower;
				manifold.pointCount += 1;
				cp += 1;
			}

			{
				cp->localAnchorA = vUpper;
				cp->separation = separationUpper;
				manifold.pointCount += 1;
			}
		}
		else
		{
			manifold.normal = -normal;
			cManifoldPoint* cp = manifold.points + 0;

			{
				cp->localAnchorA = vUpper;
				cp->separation = separationUpper;
				manifold.pointCount += 1;
				cp += 1;
			}

			{
				cp->localAnchorA = vLower;
				cp->separation = separationLower;
				manifold.pointCount += 1;
			}
		}

		return manifold;
		
	}
	
	static cManifold SATClipper(const cShape* shapeA, const cShape* shapeB)
	{
		int edgeA = 0;
		float separationA = findMaxSeparation(edgeA, shapeA, shapeB);

		int edgeB = 0;
		float separationB = findMaxSeparation(edgeB, shapeB, shapeA);
		
		bool flip;
		vec2 searchDir = vec2::zero;
		
		if (separationB > separationA) {
			flip = true;
			searchDir = shapeB->normals[edgeB];
			edgeA = findIncidentEdgeIndex(shapeA, searchDir);
			// separation B is pen depth
		}
		else {
			flip = false;
			searchDir = shapeA->normals[edgeA];
			edgeB = findIncidentEdgeIndex(shapeB, searchDir);
			// separation A is pen depth
		}

		return clipPolygons(shapeA, shapeB, edgeA, edgeB, flip);
	}
	

	cManifold CollideShapes(const cShape* shapeA, const cTransform& xfA,
							const cShape* shapeB, const cTransform& xfB,
							cGJKCache* cache)
	{
		cManifold manifold;
		cTransform identity;
		identity.SetIdentity();
		cTransform identity2;
		identity2.SetIdentity();
		cTransform xf = InvMulTransforms(xfA, xfB);
		cShape localShapeB;
		localShapeB.vertices = shapeB->vertices;
		localShapeB.normals = shapeB->normals;
		localShapeB.count = shapeB->count;
		for (int i = 0; i < localShapeB.count; ++i)
		{
			localShapeB.vertices[i] = cTransformVec(xf, shapeB->vertices[i]);
			localShapeB.normals[i] = shapeB->normals[i].rotated(xf.rot);
		}
		
		cGJKProxy gjka{ shapeA->vertices.data(), shapeA->count };
		cGJKProxy gjkb{ localShapeB.vertices.data(), localShapeB.count };
		cGJKInput input{ gjka, gjkb, identity , identity2 };
		cGJKOutput output;
		
		cGJK(input, output, cache);
		if (output.distance < 0.0005f)
			cEPA(input, output, cache);
		
		manifold.points[0].localAnchorA = output.pointA;
		manifold.points[1].localAnchorA = output.pointB;
		manifold.pointCount = 2;
		return manifold;

		//if (output.distance > 0.02f)
		//{
		//	return manifold;
		//}
		//
		//if (output.distance < 0.0005f)
		//{
		//	manifold = SATClipper(shapeA, &localShapeB);
		//	
		//	if (manifold.pointCount > 0)
		//	{
		//		manifold.normal = manifold.normal.rotated(xfA.rot);
		//		for (int i = 0; i < manifold.pointCount; ++i)
		//		{
		//			manifold.points[i].localAnchorB = cInvTransformVec(xf, manifold.points[i].localAnchorA);
		//		}
		//	}

		//	return manifold;
		//}

		//if (cache->count == 1)
		//{
		//	// vertex-vertex collision
		//	vec2 pA = output.pointA;
		//	vec2 pB = output.pointB;

		//	float distance = output.distance;
		//	vec2 normal = (pB - pA).normalized();
		//	vec2 contactPointA = pB + (0.5f * distance) * normal;

		//	manifold.normal = normal.rotated(xfA.rot);
		//	cManifoldPoint* cp = manifold.points + 0;
		//	cp->localAnchorA = contactPointA;
		//	cp->localAnchorB = cInvTransformVec(xf, contactPointA);
		//	cp->separation = distance;
		//	//cp->id = S2_MAKE_ID(cache->indexA[0], cache->indexB[0]);
		//	manifold.pointCount = 1;
		//	return manifold;
		//}

		//// vertex-edge collision
		//cassert(cache->count == 2);
		//bool flip;
		//int countA = shapeA->count;
		//int countB = localShapeB.count;
		//int edgeA, edgeB;

		//int a1 = cache->indexA[0];
		//int a2 = cache->indexA[1];
		//int b1 = cache->indexB[0];
		//int s2x = cache->indexB[1];
		//

		//if (a1 == a2)
		//{
		//	// 1 point on A, expect 2 points on B
		//	cassert(b1 != s2x);

		//	// Find reference edge that most aligns with vector between closest points.
		//	// This works for capsules and polygons
		//	vec2 axis = output.pointA - output.pointB;
		//	float dot1 = axis.dot(localShapeB.normals[b1]);
		//	float dot2 = axis.dot(localShapeB.normals[s2x]);
		//	edgeB = dot1 > dot2 ? b1 : s2x;

		//	flip = true;

		//	// Get the normal of the reference edge in polyA's frame.
		//	axis = localShapeB.normals[edgeB];

		//	// Find the incident edge on polyA
		//	// Limit search to edges adjacent to closest vertex on A
		//	int edgeA1 = a1;
		//	int edgeA2 = edgeA1 == 0 ? countA - 1 : edgeA1 - 1;
		//	dot1 = axis.dot(shapeA->normals[edgeA1]);
		//	dot2 = axis.dot(shapeA->normals[edgeA2]);
		//	edgeA = dot1 < dot2 ? edgeA1 : edgeA2;
		//}
		//else
		//{
		//	// Find reference edge that most aligns with vector between closest points.
		//	// This works for capsules and polygons
		//	vec2 axis = output.pointB - output.pointA;
		//	float dot1 = axis.dot(shapeA->normals[a1]);
		//	float dot2 = axis.dot(shapeA->normals[a2]);
		//	edgeA = dot1 > dot2 ? a1 : a2;

		//	flip = false;

		//	// Get the normal of the reference edge in polyB's frame.
		//	axis = shapeA->normals[edgeA];

		//	// Find the incident edge on polyB
		//	// Limit search to edges adjacent to closest vertex
		//	int edgeB1 = b1;
		//	int edgeB2 = edgeB1 == 0 ? countB - 1 : edgeB1 - 1;
		//	dot1 = axis.dot(localShapeB.normals[edgeB1]);
		//	dot2 = axis.dot(localShapeB.normals[edgeB2]);
		//	edgeB = dot1 < dot2 ? edgeB1 : edgeB2;
		//}

		//manifold = clipPolygons(shapeA, &localShapeB, edgeA, edgeB, flip);
		//if (manifold.pointCount > 0)
		//{
		//	manifold.normal = manifold.normal.rotated(xfA.rot); // s2RotateVector(xfA.q, manifold.normal);
		//	for (int i = 0; i < manifold.pointCount; ++i)
		//	{
		//		manifold.points[i].localAnchorB = cInvTransformVec(xf, manifold.points[i].localAnchorA);
		//	}
		//}

		//return manifold;
	}
}