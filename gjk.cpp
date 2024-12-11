
#include "pch.h"
#include "gjk.h"

namespace chiori
{
	bool floatEqual(float a, float b) { return std::abs(a - b) < HEPSILON; }
	bool compareSigns(float a, float b)
	{
		return (a > 0 && b > 0) || (a < 0 && b < 0);
	}

	vec2 GJKobject::getSupportPoint(const vec2& inDir) const
	{
		vec2 localDir = inDir.rotated(-tfm.rot * RAD2DEG);
		vec2 result = baseVertices[0];
		float maxDot = result.dot(localDir);
		for (int i = 1; i < baseVertices.size(); i++)
		{
			vec2 vertex = baseVertices[i].cmult(tfm.scale);
			float dot = vertex.dot(localDir);
			if (dot > maxDot)
			{
				maxDot = dot;
				result = vertex;
			}
		}		
		vec2 w = result.rotated(tfm.rot * RAD2DEG);
		return w + tfm.pos;
	}
	
	#pragma region Distance Subalgorithm
	std::vector<float> S1D(Simplex& simplex, boolean debugSpit = false)
	{
		const vec2& s1 = simplex[0].w;
		const vec2& s2 = simplex[1].w;
		vec2 t = s2 - s1;
		
		// orthogonal projection of the origin onto the infinite line s1s2
		vec2 p0 = s1 + (-s1.dot(t) / t.dot(t)) * t;

		// Calculate barycentric coordinates for s1 and s2 based on p0
		// Reduce to the dimension with the largest absolute value
		float mu_max = s1.x - s2.x;
		int I = 0;
		if (std::abs(s1.y - s2.y) > std::abs(mu_max)) {
			mu_max = s1.y - s2.y;
			I = 1; // Track which component is most influential
		}

		// Calculate signed area relative to the coordinate with the largest difference
		float C[2];
		C[0] = -(s2[I] - p0[I]);
		C[1] = s1[I] - p0[I];

		// Determine whether to keep the full simplex or reduce it (compare signs algo)
		bool allSignsMatch = compareSigns(C[0], mu_max) && compareSigns(C[1], mu_max);

		if (allSignsMatch)
		{
			return { C[0] / mu_max, C[1] / mu_max };
		}

		simplex = { simplex[0] };
		return { 1.0f };	
	}
	
	float computeDStar(const std::vector<float>& lambdas, const Simplex& simplex)
	{
		vec2 weightedSum = { 0.0f, 0.0f }; // Initialize the weighted sum as a 2D vector
		for (size_t i = 0; i < simplex.size(); ++i) {
			weightedSum += lambdas[i] * simplex[i].w; // Accumulate weighted points
		}
		return weightedSum.magnitude(); // Return the magnitude of the weighted sum
	}

	std::vector<float> S2D(Simplex& simplex, boolean debugSpit = false)
	{
		const vec2& s1 = simplex[0].w;
		const vec2& s2 = simplex[1].w;
		const vec2& s3 = simplex[2].w;

		// no need to calculate p0 (projection of origin onto plane), as the origin will lie in the same plane as these points
		
		// Find signed area
		// Normally we reduce to the dimension with the largest absolute value, but since
		// we are already in 2D, our vectors don't have a 3rd component to reduce,
		// so we just leave them alone (in 3D, we would have to remove the coordinate to project everything into 2D)
		// the signed area of the triangle remains the same in 2D
		float mu_max =	s1.x * (s2.y - s3.y) +
						s2.x * (s3.y - s1.y) +	
						s3.x * (s1.y - s2.y);

		// Calculate barycentric coordinates for s1, s2, and s3
		// In the paper it uses a determinant calculation, which we can simplify in 2D
		// to a simple 2D cross product.
		// Corresponds to the signed area of 2-simplex: (p0, s2, s3)
		float C1 = s2.cross(s3);
		// Corresponds to the signed area of 2-simplex: (p0, s1, s3)
		float C2 = s3.cross(s1);
		// Corresponds to the signed area of 2-simplex: (p0, s1, s2)
		float C3 = s1.cross(s2);
	
		bool cmp1 = compareSigns(mu_max, C1),
			cmp2 = compareSigns(mu_max, C2),
			cmp3 = compareSigns(mu_max, C3);
		
		if (cmp1 && cmp2 && cmp3)
		{
			// encloses origin in 2D
			return {
				C1 / mu_max,
				C2 / mu_max,
				C3 / mu_max
			};
		}

		float d = FLT_MAX; // minimum distance to origin
		std::vector<float> l; // default to s1
		Simplex w;
		if (!cmp2)
		{
			// s2 appears to be non-contributing, so we check the reduced simplex { s1. s3 }
			w = { simplex[0], simplex[2] };
			auto ls = S1D(w, debugSpit);
			float ds = computeDStar(ls, w);
			if (ds < d)
			{
				simplex = w;
				l = ls;
				d = ds;
			}
		}

		if (!cmp3)
		{
			// s3 appears to be non-contributing, so we check the reduced simplex { s1. s2 }
			w = { simplex[0], simplex[1] };
			auto ls = S1D(w, debugSpit);
			float ds = computeDStar(ls, w);
			if (ds < d)
			{
				simplex = w;
				l = ls;
				d = ds;
			}
		}

		if (!cmp1)
		{
			// s1 appears to be non-contributing, so we check the reduced simplex { s1. s2 }
			// Unlikely to reach here, as s1 is meant to be the latest point towards the origin
			// however, included for robustness
			w = { simplex[1], simplex[2] };
			auto ls = S1D(w, debugSpit);
			float ds = computeDStar(ls, w);  
			if (ds < d)
			{
				simplex = w;
				l = ls;
				d = ds;
			}
		}

		return l;
	}

	std::vector<float> signedVolumeDistanceSubalgorithm(Simplex& outSimplex, boolean debugSpit = false)
	{
		int dim = outSimplex.size() - 1;
		switch (dim)
		{
		case 1:
			return S1D(outSimplex, debugSpit);
		case 2:
			return S2D(outSimplex, debugSpit);
		default:
			return { 1.0f }; // leave the simplex as is
		}
	}
	#pragma endregion

	GJKresult GJK(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex)
	{
		std::vector<Mvert> previousPoints;
		GJKresult result;
		vec2 dir = inPrimary.tfm.pos - inTarget.tfm.pos;
		if (outSimplex.size())
			dir = outSimplex[0].w;

		for (int itr = 0; itr < commons::GJK_ITERATIONS; itr++)
		{	
			float dirm = dir.sqrMagnitude();
			Mvert w = GetSupportVertex(inPrimary, inTarget, -dir);

			if (outSimplex.isDupe(w))
				break;

			if ((dirm - dir.dot(w.w)) <= (dirm * LEPSILON * LEPSILON)) // Termination condition A
				break;

			previousPoints.push_back(w);
			outSimplex.push_front(w);
			
			std::vector<float> lambdas = signedVolumeDistanceSubalgorithm(outSimplex);

			// We determine the closest points on each shape via the barycentric coordinates
			dir = vec2::zero;
			result.z1 = vec2::zero;
			result.z2 = vec2::zero;
			for (int l = 0; l < lambdas.size(); l++) // the size of the simplex should always be the size of the lambdas
			{
				result.z1 += lambdas[l] * outSimplex[l].a;
				result.z2 += lambdas[l] * outSimplex[l].b;
				std::cout << "l" << l << ": " << lambdas[l] << std::endl;
				dir += lambdas[l] * outSimplex[l].w;
			}
				
			if (outSimplex.size() >= 3)
				break;

			//float max_norm = 1.0f;
			//for (const auto& m : outSimplex)
			//{
			//	float norm = m.w.sqrMagnitude();
			//	max_norm = (max_norm > norm) ? max_norm : norm;
			//}
			//
			//if (dir.sqrMagnitude() < (HEPSILON * max_norm)) // Termination condition B
			//	break;
		}
		
		result.distance = dir.magnitude();
		return result;
	}
	
	// Finds the closest edge to the origin on a simplex
	// Used for EPA
	Edge FindClosestEdge(const std::vector<Mvert>& poly)
	{
		Edge closest;
		closest.distance = FLT_MAX;
		for (size_t i = 0; i < poly.size(); i++)
		{
			size_t j = (i + 1) % poly.size();

			vec2 edge = poly[i].w - poly[j].w;		
			vec2 normal;

			normal = { edge.y, -edge.x };
			normal = normal.normalized();

			float distance = normal.dot(poly[i].w);

			// wrong normal, facing away from origin
			// flip normal
			if (distance < 0)
			{
				distance *= -1;
				normal *= -1;
			}

			// if this distance is closer, use this distance
			if (distance < closest.distance)
			{
				closest.distance = distance;
				closest.normal = normal;
				closest.index = (int)j;
			}
		}
		// return closest edge found
		return closest;
	}
	
	void ComputeWitnessPoints(const std::vector<Mvert>& polytope, const Edge& closestEdge, vec2& witnessA, vec2& witnessB, vec2(&c1)[2], vec2(&c2)[2])
	{
		const vec2& col_normal = closestEdge.normal;
		int I = closestEdge.index;
		const Mvert& m1 = polytope[(I - 1 + polytope.size()) % polytope.size()];
		const Mvert& m2 = polytope[I];
		vec2 edge = m2.w - m1.w;
		float lengthSqr = edge.sqrMagnitude();

		// Barycentric coordinates
		float lambda1 = (m2.w.dot(edge)) / lengthSqr;
		float lambda2 = 1.0f - lambda1;

		// Interpolate witness points
		witnessA = lambda1 * m1.a + lambda2 * m2.a;
		witnessB = lambda1 * m1.b + lambda2 * m2.b;

		c1[0] = m1.a;
		c1[1] = m2.a;
		c2[0] = m1.b;
		c2[1] = m2.b;
		
	}

	GJKresult EPA(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex, GJKresult& result)
	{
		std::vector<Mvert> polytope(outSimplex.begin(), outSimplex.end());
		Edge closestEdge;
		float supportDist = 0.0f;
		for (int i = 0; i < commons::GJK_ITERATIONS; i++)
		{
			closestEdge = FindClosestEdge(polytope);

			Mvert support = GetSupportVertex(inPrimary, inTarget, closestEdge.normal);

			supportDist = support.w.dot(closestEdge.normal);

			if (abs(supportDist - closestEdge.distance) < EPSILON)
				break;

			polytope.insert(polytope.begin() + closestEdge.index, support);
		}
		result.normal = closestEdge.normal;
		result.intersection_distance = -supportDist;
		ComputeWitnessPoints(polytope, closestEdge, result.z1, result.z2, result.c1, result.c2);
		return result;
	}
	
	GJKresult CollisionDetection(const GJKobject& inPrimary, const GJKobject& inTarget)
	{
		Simplex s;
		GJKresult result = GJK(inPrimary, inTarget, s);
		result.s = s;
		if (result.distance <= EPSILON && s.size() > 1)
		{
			result = EPA(inPrimary, inTarget, s, result);
		}
		return result;
	}





	
	struct cSimplexVertex
	{
		vec2 wA;		// support point in proxyA
		vec2 wB;		// support point in proxyB
		vec2 w;		// wB - wA
		float a;		// barycentric coordinate for closest point
		int indexA;	// wA index
		int indexB;	// wB index
	};

	struct cSimplex
	{
		cSimplex(const cGJKProxy* proxyA, const cTransform& transformA, const cGJKProxy* proxyB, const cTransform& transformB)
		{
			cSimplexVertex* vertices = &m_v1;
			cSimplexVertex* v = vertices + 0;
			v->indexA = 0;
			v->indexB = 0;
			vec2 wALocal = proxyA->GetVertex(0);
			vec2 wBLocal = proxyB->GetVertex(0);
			v->wA = cTransformVec(transformA, wALocal);
			v->wB = cTransformVec(transformB, wBLocal);
			v->w = v->wB - v->wA;
			v->a = 1.0f;
			m_count = 1;
		}
		
		vec2 GetSearchDirection() const
		{
			switch (m_count)
			{
			case 1:
				return -m_v1.w;

			case 2:
			{
				vec2 e12 = m_v2.w - m_v1.w;
				float sgn = cross(e12, -m_v1.w);
				if (sgn > 0.0f)
				{
					// Origin is left of e12.
					return cross(1.0f, e12);
				}
				else
				{
					// Origin is right of e12.
					return cross(e12, 1.0f);
				}
			}

			default:
				cassert(false);
				return vec2::zero;
			}
		}

		vec2 GetClosestPoint() const
		{
			switch (m_count)
			{
			case 0:
				cassert(false);
				return vec2::zero;

			case 1:
				return m_v1.w;

			case 2:
				return m_v1.a * m_v1.w + m_v2.a * m_v2.w;

			case 3:
				return vec2::zero;

			default:
				cassert(false);
				return vec2::zero;
			}
		}

		void GetWitnessPoints(vec2* pA, vec2* pB) const
		{
			switch (m_count)
			{
			case 0:
				cassert(false);
				break;

			case 1:
				*pA = m_v1.wA;
				*pB = m_v1.wB;
				break;

			case 2:
				*pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
				*pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
				break;

			case 3:
				*pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
				*pB = *pA;
				break;

			default:
				cassert(false);
				break;
			}
		}

		void S1D();
		void S2D();

		cSimplexVertex m_v1, m_v2, m_v3;
		int m_count;
	};


	// Solve a line segment using barycentric coordinates.
	//
	// p = a1 * w1 + a2 * w2
	// a1 + a2 = 1
	//
	// The vector from the origin to the closest point on the line is
	// perpendicular to the line.
	// e12 = w2 - w1
	// dot(p, e) = 0
	// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
	//
	// 2-by-2 linear system
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	//
	// Define
	// d12_1 =  dot(w2, e12)
	// d12_2 = -dot(w1, e12)
	// d12 = d12_1 + d12_2
	//
	// Solution
	// a1 = d12_1 / d12
	// a2 = d12_2 / d12
	void cSimplex::S1D()
	{
		vec2 w1 = m_v1.w;
		vec2 w2 = m_v2.w;
		vec2 e12 = w2 - w1;

		// w1 region
		float d12_2 = -dot(w1, e12);
		if (d12_2 <= 0.0f)
		{
			// a2 <= 0, so we clamp it to 0
			m_v1.a = 1.0f;
			m_count = 1;
			return;
		}

		// w2 region
		float d12_1 = dot(w2, e12);
		if (d12_1 <= 0.0f)
		{
			// a1 <= 0, so we clamp it to 0
			m_v2.a = 1.0f;
			m_count = 1;
			m_v1 = m_v2;
			return;
		}

		// Must be in e12 region.
		float inv_d12 = 1.0f / (d12_1 + d12_2);
		m_v1.a = d12_1 * inv_d12;
		m_v2.a = d12_2 * inv_d12;
		m_count = 2;
	}

	// Possible regions:
	// - points[2]
	// - edge points[0]-points[2]
	// - edge points[1]-points[2]
	// - inside the triangle
	void cSimplex::S2D()
	{
		vec2 w1 = m_v1.w;
		vec2 w2 = m_v2.w;
		vec2 w3 = m_v3.w;

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		vec2 e12 = w2 - w1;
		float w1e12 = dot(w1, e12);
		float w2e12 = dot(w2, e12);
		float d12_1 = w2e12;
		float d12_2 = -w1e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		vec2 e13 = w3 - w1;
		float w1e13 = dot(w1, e13);
		float w3e13 = dot(w3, e13);
		float d13_1 = w3e13;
		float d13_2 = -w1e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		vec2 e23 = w3 - w2;
		float w2e23 = dot(w2, e23);
		float w3e23 = dot(w3, e23);
		float d23_1 = w3e23;
		float d23_2 = -w2e23;

		// Triangle123
		float n123 = cross(e12, e13);

		float d123_1 = n123 * cross(w2, w3);
		float d123_2 = n123 * cross(w3, w1);
		float d123_3 = n123 * cross(w1, w2);

		// w1 region
		if (d12_2 <= 0.0f && d13_2 <= 0.0f)
		{
			m_v1.a = 1.0f;
			m_count = 1;
			return;
		}

		// e12
		if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
		{
			float inv_d12 = 1.0f / (d12_1 + d12_2);
			m_v1.a = d12_1 * inv_d12;
			m_v2.a = d12_2 * inv_d12;
			m_count = 2;
			return;
		}

		// e13
		if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
		{
			float inv_d13 = 1.0f / (d13_1 + d13_2);
			m_v1.a = d13_1 * inv_d13;
			m_v3.a = d13_2 * inv_d13;
			m_count = 2;
			m_v2 = m_v3;
			return;
		}

		// w2 region
		if (d12_1 <= 0.0f && d23_2 <= 0.0f)
		{
			m_v2.a = 1.0f;
			m_count = 1;
			m_v1 = m_v2;
			return;
		}

		// w3 region
		if (d13_1 <= 0.0f && d23_1 <= 0.0f)
		{
			m_v3.a = 1.0f;
			m_count = 1;
			m_v1 = m_v3;
			return;
		}

		// e23
		if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
		{
			float inv_d23 = 1.0f / (d23_1 + d23_2);
			m_v2.a = d23_1 * inv_d23;
			m_v3.a = d23_2 * inv_d23;
			m_count = 2;
			m_v1 = m_v3;
			return;
		}

		// Must be in triangle123
		float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
		m_v1.a = d123_1 * inv_d123;
		m_v2.a = d123_2 * inv_d123;
		m_v3.a = d123_3 * inv_d123;
		m_count = 3;
	}


	cGJKOutput cGJK(const cGJKInput& input)
	{
		cGJKOutput output;
		const cGJKProxy* proxyA = &input.proxyA;
		const cGJKProxy* proxyB = &input.proxyB;

		cTransform transformA = input.transformA;
		cTransform transformB = input.transformB;

		// Initialize the simplex.
		cSimplex simplex( proxyA, transformA, proxyB, transformB );

		// Get simplex vertices as an array.
		cSimplexVertex* vertices = &simplex.m_v1;
		const int k_maxIters = commons::GJK_ITERATIONS;

		// These store the vertices of the last simplex so that we
		// can check for duplicates and prevent cycling.
		int saveA[3], saveB[3];
		int saveCount = 0;

		float distanceSqr1 = FLT_MAX;
		float distanceSqr2 = distanceSqr1;

		// Main iteration loop.
		int iter = 0;
		while (iter < k_maxIters)
		{
			// Copy simplex so we can identify duplicates.
			saveCount = simplex.m_count;
			for (int i = 0; i < saveCount; ++i)
			{
				saveA[i] = vertices[i].indexA;
				saveB[i] = vertices[i].indexB;
			}

			switch (simplex.m_count)
			{
			case 1:
				break;

			case 2:
				simplex.S1D();
				break;

			case 3:
				simplex.S2D();
				break;

			default:
				cassert(false);
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if (simplex.m_count == 3)
			{
				break;
			}

			// Compute closest point.
			vec2 p = simplex.GetClosestPoint();
			distanceSqr2 = p.sqrMagnitude();

			// Ensure progress
			if (distanceSqr2 >= distanceSqr1)
			{
				//break;
			}
			distanceSqr1 = distanceSqr2;

			// Get search direction.
			vec2 d = simplex.GetSearchDirection();

			// Ensure the search direction is numerically fit.
			if (d.sqrMagnitude() < EPSILON * EPSILON)
			{
				// The origin is probably contained by a line segment
				// or triangle. Thus the shapes are overlapped.

				// We can't return zero here even though there may be overlap.
				// In case the simplex is a point, segment, or triangle it is difficult
				// to determine if the origin is contained in the CSO or very close to it.
				break;
			}

			// Compute a tentative new simplex vertex using support points.
			cSimplexVertex* vertex = vertices + simplex.m_count;
			vertex->indexA = proxyA->getSupport((-d).rotate(transformA.rot));
			vertex->wA = cTransformVec(transformA, proxyA->GetVertex(vertex->indexA));
			vec2 wBLocal;
			vertex->indexB = proxyB->getSupport((d).rotate(transformB.rot));
			vertex->wB = cTransformVec(transformB, proxyB->GetVertex(vertex->indexB));
			vertex->w = vertex->wB - vertex->wA;

			// Iteration count is equated to the number of support point calls.
			++iter;

			// Check for duplicate support points. This is the main termination criteria.
			bool duplicate = false;
			for (int i = 0; i < saveCount; ++i)
			{
				if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
				{
					duplicate = true;
					break;
				}
			}

			// If we found a duplicate support point we must exit to avoid cycling.
			if (duplicate)
			{
				break;
			}

			// New vertex is ok and needed.
			++simplex.m_count;
		}

		// Prepare output.
		simplex.GetWitnessPoints(&output.pointA, &output.pointB);
		output.distance = distance(output.pointA, output.pointB);
		output.iterations = iter;

		return output;
	}


	cGJKOutput cDistance(const cGJKProxy& inPrimary, cTransform inPrimaryTransform,
		const cGJKProxy& inTarget, cTransform inTargetTransform)
	{
		cGJKInput input;
		input.proxyA = inPrimary;
		input.proxyB = inTarget;
		input.transformA = inPrimaryTransform;
		input.transformB = inTargetTransform;

		return cGJK(input);	
	}
}