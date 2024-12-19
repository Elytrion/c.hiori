#include "pch.h"
#include "manifold.h"
#include "gjk.h"
#include "cShape.h"

#include "cprocessing.h" //!!TO REMOVE!!

namespace chiori
{
	#define clinearSlop 0.005f
	#define cspeculativeDistance (4.0f * clinearSlop)


	static std::vector<vec2> clipEdge(vec2& v1, vec2& v2, vec2& n, float o)
	{
		std::vector<vec2> result;
		float d1 = n.dot(v1) - o;
		float d2 = n.dot(v2) - o;
		// if either point is past o along n
		// then we can keep the point
		if (d1 >= 0.0) result.push_back(v1);
		if (d2 >= 0.0) result.push_back(v2);
		// finally we need to check if they
		// are on opposing sides so that we can
		// compute the correct point
		if (d1 * d2 < 0.0) {
			// if they are on different sides of the
			// offset, d1 and d2 will be a (+) * (-)
			// and will yield a (-) and therefore be
			// less than zero
			// get the vector for the edge we are clipping
			vec2 e = v2 - v1;
			// compute the location along e
			float u = d1 / (d1 - d2);
			e *= u;
			e += v1;
			// add the point
			result.push_back(e);
		}

		return result;
	}

	static cManifold PolygonClipper(const cPolygon* polyA, const cPolygon* polyB, int edgeA, int edgeB, bool flip)
	{
		//std::cout << "CLIPPING POLYS" << std::endl;
		//std::cout << "POLYA" << std::endl;
		//std::cout << "Verts: " << std::endl;
		//for (int i = 0; i < polyA->count; i++)
		//{
		//	std::cout << polyA->vertices[i] << " , ";
		//}
		//std::cout << "\nNorms: " << std::endl;
		//for (int i = 0; i < polyA->count; i++)
		//{
		//	std::cout << polyA->normals[i] << " , ";
		//}
		//std::cout << "POLYB" << std::endl;
		//std::cout << "Verts: " << std::endl;
		//for (int i = 0; i < polyB->count; i++)
		//{
		//	std::cout << polyB->vertices[i] << " , ";
		//}
		//std::cout << "\nNorms: " << std::endl;
		//for (int i = 0; i < polyB->count; i++)
		//{
		//	std::cout << polyB->normals[i] << " , ";
		//}
		//std::cout << "\nEdgeA: " << edgeA << std::endl;
		//std::cout << "EdgeB: " << edgeB << std::endl;
		//std::cout << "Flipped? " << flip << std::endl;
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
		//std::cout << "Ref edge: " << v11 << " + " << v12 << std::endl;
		// Incident edge vertices
		vec2 v21 = poly2->vertices[i21];
		vec2 v22 = poly2->vertices[i22];
		//std::cout << "Inc edge: " << v21 << " + " << v22 << std::endl;

		vec2 refv = (v12 - v11);
		refv.normalize();
		float offset1 = refv.dot(v11);
		std::vector<vec2> cpts1 = clipEdge(v21, v22, refv, offset1);
		if (cpts1.size() < 2)
		{
			//std::cout << " FIRST CLIPPING FAILED! " << std::endl;
			return manifold;
		}
		float offset2 = refv.dot(v12);
		std::vector<vec2> cpts = clipEdge(cpts1[0], cpts1[1], -refv, -offset2);
		if (cpts.size() < 2)
		{
			//std::cout << " SECOND CLIPPING FAILED! " << std::endl;
			return manifold;
		}

		// Compute the maximum penetration depth
		float maxDepth = normal.dot(v11);

		// Validate clipped points and populate the manifold
		for (size_t i = 0; i < cpts.size(); ++i)
		{
			float separation = normal.dot(cpts[i]) - maxDepth;
			if (separation <= 0.0f && manifold.pointCount < 2)
			{
				cManifoldPoint& mp = manifold.points[manifold.pointCount++];
				mp.localAnchorA = cpts[i];
				mp.separation = separation;
			}
		}
		
		manifold.normal = flip ? -normal : normal;
		//if (manifold.pointCount == 0)
		//{
		//	std::cout << "ERROR! NO CONTACT POINTS ADDED" << std::endl;
		//}
		return manifold;
	}
	
	// Polygon clipper used by GJK and SAT to compute contact points when there are potentially two contact points.
	static cManifold ClipPolygons(const cPolygon* polyA, const cPolygon* polyB, int edgeA, int edgeB, bool flip)
	{
		//std::cout << "CLIPPING POLYS" << std::endl;
		//std::cout << "POLYA" << std::endl;
		//std::cout << "Verts: " << std::endl;
		//for (int i = 0; i < polyA->count; i++)
		//{
		//	std::cout << polyA->vertices[i] << " , ";
		//}
		//std::cout << "\nNorms: " << std::endl;
		//for (int i = 0; i < polyA->count; i++)
		//{
		//	std::cout << polyA->normals[i] << " , ";
		//}
		//std::cout << "POLYB" << std::endl;
		//std::cout << "Verts: " << std::endl;
		//for (int i = 0; i < polyB->count; i++)
		//{
		//	std::cout << polyB->vertices[i] << " , ";
		//}
		//std::cout << "\nNorms: " << std::endl;
		//for (int i = 0; i < polyB->count; i++)
		//{
		//	std::cout << polyB->normals[i] << " , ";
		//}
		//std::cout << "\nEdgeA: " << edgeA << std::endl;
		//std::cout << "EdgeB: " << edgeB << std::endl;
		//std::cout << "Flipped? " << flip << std::endl;
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
		//std::cout << "Ref edge: " << v11 << " + " << v12 << std::endl;
		// Incident edge vertices
		vec2 v21 = poly2->vertices[i21];
		vec2 v22 = poly2->vertices[i22];
		//std::cout << "Inc edge: " << v21 << " + " << v22 << std::endl;
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
	static float FindMaxSeparation(int* edgeIndex, const cPolygon* poly1, const cPolygon* poly2)
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
	static cManifold PolygonSAT(const cPolygon* polyA, const cPolygon* polyB)
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
		return PolygonClipper(polyA, polyB, edgeA, edgeB, flip);
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
			manifold = PolygonSAT(shapeA, &localShapeB);
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