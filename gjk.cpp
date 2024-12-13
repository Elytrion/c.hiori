
#include "pch.h"
#include "gjk.h"

namespace chiori
{
	bool compareSigns(float a, float b)
	{
		return (a > 0 && b > 0) || (a < 0 && b < 0);
	}
	
	struct cSimplexVertex
	{
		vec2 wA{ vec2::zero };		// support point in proxyA
		vec2 wB{ vec2::zero };		// support point in proxyB
		vec2 w{ vec2::zero };			// wA - wB
		float l{ -1.0f };		// barycentric lambda for closest point
		int indexA{ -1 };		// wA index
		int indexB{ -1 };		// wB index
	};

	struct cSimplex
	{
		float GetMetric() const
		{
			switch (s_size)
			{
			case 0:
				cassert(false);
				return 0.0;

			case 1:
				return 0.0f;

			case 2:
				return distance(m1.w, m2.w);

			case 3:
				return cross(m2.w - m1.w, m3.w - m1.w);

			default:
				cassert(false);
				return 0.0f;
			}
		}

		void ReadCache(const cGJKCache* cache,
			const cGJKProxy& proxyA, const cTransform& transformA,
			const cGJKProxy& proxyB, const cTransform& transformB)
		{
			cassert(cache);
			cassert(cache->count <= 3);
			cSimplexVertex* vertices = &m1;

			// Copy data from cache.
			s_size = cache->count;
			for (int i = 0; i < s_size; ++i)
			{
				cSimplexVertex* v = vertices + i;
				v->indexA = cache->indexA[i];
				v->indexB = cache->indexB[i];
				vec2 wALocal = proxyA.GetVertex(v->indexA);
				vec2 wBLocal = proxyB.GetVertex(v->indexB);
				v->wA = cTransformVec(transformA, wALocal);
				v->wB = cTransformVec(transformB, wBLocal);
				v->w = v->wA - v->wB;
				v->l = 0.0f;
			}

			// Compute the new simplex metric, if it is substantially different than
			// old metric then flush the simplex.
			if (s_size > 1)
			{
				float metric1 = cache->metric;
				float metric2 = GetMetric();
				if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < EPSILON)
				{
					// Reset the simplex.
					s_size = 0;
				}
			}
			
			// If the cache is empty or invalid ...
			if (s_size == 0)
			{
				m1.indexA = 0;
				m1.indexB = 0;
				m1.wA = cTransformVec(transformA, proxyA.GetVertex(0));
				m1.wB = cTransformVec(transformB, proxyB.GetVertex(0));
				m1.w = m1.wA - m1.wB;
				m1.l = 1.0f;
				s_size = 1;
			}
		}

		void WriteCache(cGJKCache* cache) const
		{
			cassert(cache);
			
			cache->metric = GetMetric();
			cache->count = static_cast<unsigned>(s_size);
			const cSimplexVertex* vertices = &m1;
			for (int i = 0; i < s_size; ++i)
			{
				cache->indexA[i] = static_cast<unsigned>(vertices[i].indexA);
				cache->indexB[i] = static_cast<unsigned>(vertices[i].indexB);
			}
		}
		
		cSimplexVertex m1, m2, m3;
		int s_size;
	};
	
	#pragma region Distance Subalgorithm
	void S1D(cSimplex& simplex)
	{
		cSimplexVertex& m1 = simplex.m1;
		cSimplexVertex& m2 = simplex.m2;
		
		const vec2& s1 = m1.w;
		const vec2& s2 = m2.w;
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
		float C1 = -(s2[I] - p0[I]);
		float C2 = s1[I] - p0[I];

		// Determine whether to keep the full simplex or reduce it (compare signs algo)
		bool allSignsMatch = compareSigns(C1, mu_max) && compareSigns(C2, mu_max);

		if (allSignsMatch)
		{
			m1.l = C1 / mu_max;
			m2.l = C2 / mu_max;
			return;
		}
		else
		{
			simplex.s_size = 1;
			m1.l = 1.0f;
			return;
		}
	}
	
	float computeDStar(const cSimplex& simplex)
	{
		vec2 weightedSum = vec2::zero; // Initialize the weighted sum as a 2D vector
		const cSimplexVertex* verts = &simplex.m1;
		for (size_t i = 0; i < simplex.s_size; ++i) {
			weightedSum += verts[i].l * verts[i].w; // Accumulate weighted points
		}
		return weightedSum.magnitude(); // Return the magnitude of the weighted sum
	}

	void S2D(cSimplex& simplex, boolean debugSpit = false)
	{
		cSimplexVertex& m1 = simplex.m1;
		cSimplexVertex& m2 = simplex.m2;
		cSimplexVertex& m3 = simplex.m3;
		
		const vec2& s1 = m1.w;
		const vec2& s2 = m2.w;
		const vec2& s3 = m3.w;

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
			m1.l = C1 / mu_max;
			m2.l = C2 / mu_max;
			m3.l = C3 / mu_max;
			return;
		}

		float d = FLT_MAX; // minimum distance to origin
		cSimplex w;
		if (!cmp2)
		{
			// s2 appears to be non-contributing, so we check the reduced simplex { s1. s3 }
			w.m1 = m1;//simplex[0];
			w.m2 = m3;// simplex[2];
			w.s_size = 2;
			S1D(w);
			float ds = computeDStar(w);
			if (ds < d)
			{
				simplex = w;
				d = ds;
			}
		}

		if (!cmp3)
		{
			// s3 appears to be non-contributing, so we check the reduced simplex { s1. s2 }
			w.m1 = m1;// simplex[0];
			w.m2 = m2;// simplex[1];
			w.s_size = 2;
			S1D(w);
			float ds = computeDStar(w);
			if (ds < d)
			{
				simplex = w;
				d = ds;
			}
		}

		if (!cmp1)
		{
			// s1 appears to be non-contributing, so we check the reduced simplex { s1. s2 }
			// Unlikely to reach here, as s1 is meant to be the latest point towards the origin
			// however, included for robustness
			w.m1 = m2;//simplex[1];
			w.m2 = m3;//simplex[2];
			w.s_size = 2;
			S1D(w);
			float ds = computeDStar(w);  
			if (ds < d)
			{
				simplex = w;
				d = ds;
			}
		}

		return;
	}

	void signedVolumeDistanceSubalgorithm(cSimplex& outSimplex)
	{
		int dim = outSimplex.s_size - 1;
		switch (dim)
		{
		case 1:
			S1D(outSimplex);
			return;
		case 2:
			S2D(outSimplex);
			return;
		default:
			return; // leave the simplex as is
		}
	}
	#pragma endregion

	void GJK(const cGJKInput& input, cGJKOutput& output, cGJKCache* cache)
	{
		const cGJKProxy& proxyA = input.proxyA;
		const cGJKProxy& proxyB = input.proxyA;
		cTransform xfA = input.transformA;
		cTransform xfB = input.transformB;
		const int max_itrs = input.maxIterations;
		const float eps = input.tolerance;	
		cSimplex simplex;
		simplex.ReadCache(cache, proxyA, xfA, proxyB, xfB);
		vec2 dir = vec2::zero;
		int saveA[3], saveB[3];
		int saveCount = 0;
		cSimplexVertex* simplexVerts = &simplex.m1;
		int itr;
		for (itr = 0; itr < max_itrs; itr++)
		{				
			saveCount = simplex.s_size;
			for (int i = 0; i < saveCount; ++i)
			{
				saveA[i] = simplexVerts[i].indexA;
				saveB[i] = simplexVerts[i].indexB;
			}
			
			signedVolumeDistanceSubalgorithm(simplex);
			// We determine the closest points on each shape via the barycentric coordinates
			dir = vec2::zero;
			for (int l = 0; l < simplex.s_size; l++) // the size of the simplex should always be the size of the lambdas
				dir += simplexVerts[l].l * simplexVerts[l].w;
				
			if (simplex.s_size >= 3)
				break;

			float dirm = dir.sqrMagnitude();
			cSimplexVertex w;
			vec2 localDirA = (-dir).rotated(-xfA.rot);
			vec2 localDirB = (dir).rotated(-xfB.rot);
			w.indexA = proxyA.getSupport(localDirA);
			w.indexB = proxyB.getSupport(localDirB);
			vec2 localVertA = proxyA.GetVertex(w.indexA);
			vec2 localVertB = proxyB.GetVertex(w.indexB);
			w.wA = cTransformVec(xfA, localVertA);
			w.wB = cTransformVec(xfB, localVertB);
			w.w = w.wA - w.wB;

			bool duplicate = false;
			// Termination condition B, duplicate support points. This is the main termination criteria.
			for (int i = 0; i < saveCount; ++i)
			{
				if (w.indexA == saveA[i] && w.indexB == saveB[i])
				{
					duplicate = true;
					break;
				}
			}
			// If we found a duplicate support point we must exit to avoid cycling.
			if (duplicate)
				break;
			// Termination condition A
			if ((dirm - dir.dot(w.w)) <= (dirm * eps * eps)) 
				break;
			// push w to the front of simplex, pushing all the values back 1
			// this ensures m1 is always the latest point, required for algorithm efficiency
			simplex.m3 = simplex.m2;
			simplex.m2 = simplex.m1;
			simplex.m1 = w;
			simplex.s_size++;
		}
		output.pointA = vec2::zero;
		output.pointB = vec2::zero;
		for (int l = 0; l < simplex.s_size; l++)
		{
			output.pointA += simplexVerts[l].l * simplexVerts[l].wA;
			output.pointB += simplexVerts[l].l * simplexVerts[l].wB;
		}
		simplex.WriteCache(cache);
		output.distance = dir.magnitude();
		output.iterations = itr;
	}
}