
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
		vec2 localDir = inDir.rotated(-rotation * RAD2DEG);
		//float rad = rotation * commons::DEG2RAD;
		//float cos_t = cos(-rad);
		//float sin_t = sin(-rad);
		//vec2 localDir = {
		//	inDir.x * cos_t - inDir.y * sin_t,
		//	inDir.x * sin_t + inDir.y * cos_t
		//};

		vec2 l_w = supportFunc(localDir);
		vec2 w = l_w.rotated(rotation * RAD2DEG);
		//cos_t = cos(rad);
		//sin_t = sin(rad);
		//vec2 w = {
		//	l_w.x * cos_t - l_w.y * sin_t,
		//	l_w.x * sin_t + l_w.y * cos_t
		//};
		return w + position;
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
		vec2 dir = inPrimary.position - inTarget.position;
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

			//We determine the closest points on each shape via the barycentric coordinates
			dir = vec2::zero;
			result.z1 = vec2::zero;
			result.z2 = vec2::zero;
			for (int l = 0; l < lambdas.size(); l++) // the size of the simplex should always be the size of the lambdas
			{
				result.z1 += lambdas[l] * outSimplex[l].a;
				result.z2 += lambdas[l] * outSimplex[l].b;
				dir += lambdas[l] * outSimplex[l].w;
			}

				
			if (outSimplex.size() >= 3)
				break;

			float max_norm = 1.0f;
			for (const auto& m : outSimplex)
			{
				float norm = m.w.sqrMagnitude();
				max_norm = (max_norm > norm) ? max_norm : norm;
			}
			
			if (dir.sqrMagnitude() < (HEPSILON * max_norm)) // Termination condition B
				break;
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
		int I = closestEdge.index;
		const Mvert& v1 = polytope[(I - 1 + polytope.size()) % polytope.size()];
		const Mvert& v2 = polytope[I];
		vec2 edge = v2.w - v1.w;
		float lengthSqr = edge.sqrMagnitude();

		// Barycentric coordinates
		float lambda1 = (v2.w.dot(edge)) / lengthSqr;
		float lambda2 = 1.0f - lambda1;

		// Interpolate witness points
		witnessA = lambda1 * v1.a + lambda2 * v2.a;
		witnessB = lambda1 * v1.b + lambda2 * v2.b;

		c1[0] = v1.a;
		c1[1] = v2.a;
		c2[0] = v1.b;
		c2[1] = v2.b;
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
}