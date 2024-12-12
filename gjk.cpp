
#include "pch.h"
#include "gjk.h"

namespace chiori
{
	bool compareSigns(float a, float b)
	{
		return (a > 0 && b > 0) || (a < 0 && b < 0);
	}

	vec2 GJKobject::getSupportPoint(const vec2& inDir) const
	{
		vec2 localDir = inDir.rotated(-tfm.rot);
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
		vec2 w = result.rotated(tfm.rot);
		return w + tfm.pos;
	}

	vec2 getLocalDir(cTransform xf, const vec2& inDir)
	{
		return inDir.rotated(-xf.rot);
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
		vec2 w;			// wB - wA
		float l;		// barycentric lambda for closest point
		int indexA;		// wA index
		int indexB;		// wB index
	};

	struct cSimplex
	{
		float GetMetric() const
		{
			switch (m_count)
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
			cassert(cache->count <= 3);

			// Copy data from cache.
			m_count = cache->count;
			cSimplexVertex* vertices = &m1;
			for (int i = 0; i < m_count; ++i)
			{
				cSimplexVertex* v = vertices + i;
				v->indexA = cache->indexA[i];
				v->indexB = cache->indexB[i];
				vec2 wALocal = proxyA.GetVertex(v->indexA);
				vec2 wBLocal = proxyB.GetVertex(v->indexB);
				v->wA = cTransformVec(transformA, wALocal);
				v->wB = cTransformVec(transformB, wBLocal);
				v->w = v->wB - v->wA;
				v->l = 0.0f;
			}

			// Compute the new simplex metric, if it is substantially different than
			// old metric then flush the simplex.
			if (m_count > 1)
			{
				float metric1 = cache->metric;
				float metric2 = GetMetric();
				if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < EPSILON)
				{
					// Reset the simplex.
					m_count = 0;
				}
			}

			// If the cache is empty or invalid ...
			if (m_count == 0)
			{
				cSimplexVertex* v = vertices;
				v->indexA = 0;
				v->indexB = 0;
				vec2 wALocal = proxyA.GetVertex(0);
				vec2 wBLocal = proxyB.GetVertex(0);
				v->wA = cTransformVec(transformA, wALocal);
				v->wB = cTransformVec(transformB, wBLocal);
				v->w = v->wB - v->wA;
				m_count = 1;
			}
		}

		void WriteCache(cGJKCache* cache) const
		{
			cache->metric = GetMetric();
			cache->count = static_cast<unsigned>(m_count);
			const cSimplexVertex* vertices = &m1;
			for (int i = 0; i < m_count; ++i)
			{
				cache->indexA[i] = static_cast<unsigned>(vertices[i].indexA);
				cache->indexB[i] = static_cast<unsigned>(vertices[i].indexB);
			}
		}

		cSimplexVertex m1, m2, m3;
		int m_count;
	};

	#pragma region Distance Subalgorithm
	void S1D(cSimplex& simplex)
	{
		const vec2& s1 = simplex.m1.w;
		const vec2& s2 = simplex.m2.w;
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
			simplex.m1.l = C[0] / mu_max;
			simplex.m2.l = C[1] / mu_max;
		}
		else
		{
			simplex.m1.l = 1.0f;
			simplex.m_count = 1;
		}
	}

	float computeDStar(const cSimplex& simplex)
	{
		vec2 weightedSum = { 0.0f, 0.0f }; // Initialize the weighted sum as a 2D vector
		const cSimplexVertex* v = &simplex.m1;
		for (size_t i = 0; i < simplex.m_count; ++i) {
			weightedSum += v->w * v->l; // Accumulate weighted points
		}
		return weightedSum.magnitude(); // Return the magnitude of the weighted sum
	}

	void S2D(cSimplex& simplex)
	{
		const vec2& s1 = simplex.m1.w;
		const vec2& s2 = simplex.m2.w;
		const vec2& s3 = simplex.m3.w;

		// no need to calculate p0 (projection of origin onto plane), as the origin will lie in the same plane as these points

		// Find signed area
		// Normally we reduce to the dimension with the largest absolute value, but since
		// we are already in 2D, our vectors don't have a 3rd component to reduce,
		// so we just leave them alone (in 3D, we would have to remove the coordinate to project everything into 2D)
		// the signed area of the triangle remains the same in 2D
		float mu_max = s1.x * (s2.y - s3.y) +
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
			simplex.m1.l = C1 / mu_max;
			simplex.m2.l = C2 / mu_max;
			simplex.m3.l = C3 / mu_max;
			return;
		}

		float d = FLT_MAX; // minimum distance to origin
		std::vector<float> l; // default to s1
		cSimplex w;
		if (!cmp2)
		{
			// s2 appears to be non-contributing, so we check the reduced simplex { s1. s3 }
			w.m_count = 2;
			w.m1 = simplex.m1;
			w.m2 = simplex.m3;
			S1D(w); // modify simplex using S1D to obtain barycentric lambdas
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
			w.m_count = 2;
			w.m1 = simplex.m1;
			w.m2 = simplex.m2;
			S1D(w); // modify simplex using S1D to obtain barycentric lambdas
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
			w.m_count = 2;
			w.m1 = simplex.m2;
			w.m2 = simplex.m3;
			S1D(w);	// modify simplex using S1D to obtain barycentric lambdas
			float ds = computeDStar(w);
			if (ds < d)
			{
				simplex = w;
				d = ds;
			}
		}
	}

	void signedVolumeDistanceSubalgorithm(cSimplex& outSimplex)
	{
		int dim = outSimplex.m_count - 1;
		switch (dim)
		{
		case 1:
			 S1D(outSimplex);
			 break;
		case 2:
			S2D(outSimplex);
			break;
		default:
			break;
		}
	}
	#pragma endregion
	

	void SetSupportVertex(cSimplex& s, const vec2& dir,
		const cGJKProxy& proxyA, const cGJKProxy& proxyB,
		const cTransform& transformA, const cTransform& transformB)
	{
		cSimplexVertex* vertex = &s.m1 + s.m_count;
		vec2 localDirA = (-dir).rotated(transformA.rot);
		vertex->indexA = proxyA.getSupport(-localDirA);
		vertex->wA = proxyA.getSupportVert(dir, transformA);
		vec2 localDirB = dir.rotated(transformB.rot);
		vertex->indexB = proxyB.getSupport(localDirB);
		vertex->wB = proxyB.getSupportVert(dir, transformB);
		vertex->w = vertex->wB - vertex->wA;
	}

	void cGJK(cGJKOutput& output, const cGJKInput& input, cGJKCache* cache)
	{
		const cGJKProxy& proxyA = input.proxyA;
		const cGJKProxy& proxyB = input.proxyB;
		vec2& closestPtA = output.pointA;
		vec2& closestPtB = output.pointB;
		cTransform transformA = input.transformA;
		cTransform transformB = input.transformB;
		float tolerance = input.tolerance;

		cSimplex simplex;
		simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB); // inits the first point in the simplex
		const int _maxItr = input.maxIterations;
		cSimplexVertex* vertices = &simplex.m1;
		vec2 dir = vec2::zero;
		// These store the vertices of the last simplex so that we
		// can check for duplicates and prevent cycling.
		int saveA[3], saveB[3];
		int saveCount = 0;
		int itr = 0;
		while (itr < _maxItr)
		{
			saveCount = simplex.m_count;
			for (int i = 0; i < saveCount; ++i)
			{
				saveA[i] = vertices[i].indexA;
				saveB[i] = vertices[i].indexB;
			}

			signedVolumeDistanceSubalgorithm(simplex);

			if (simplex.m_count >= 3) // termination condition A1
				break;

			dir = vec2::zero;
			for (int i = 0; i < simplex.m_count; i++)
			{
				dir += vertices[i].l * vertices[i].w;
			}

			float dirm = dir.sqrMagnitude();

			// Get new support vertex (at here, we know m_count < 3)
			SetSupportVertex(simplex, proxyA, proxyB, )
			cSimplexVertex* vertex = vertices + simplex.m_count;
			vertex->indexA = proxyA.getSupport((-dir).rotated(transformA.rot));
			vertex->wA = cTransformVec(transformA, proxyA.GetVertex(vertex->indexA));
			vertex->indexB = proxyB.getSupport((dir).rotated(transformB.rot));
			vertex->wB = cTransformVec(transformB, proxyB.GetVertex(vertex->indexB));
			vertex->w = vertex->wB - vertex->wA;

			++itr;

			bool duplicate = false;
			// Termination condition B, duplicate support points. This is the main termination criteria.
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
				break;
			
			if ((dirm - dir.dot(vertex->w)) <= (dirm * tolerance * tolerance)) // Termination condition C, similar to A2
				break;

			// Add the new vertex
			simplex.m_count++;
		}
		closestPtA = closestPtB = vec2::zero;
		for (int i = 0; i < simplex.m_count; i++) 
		{
			closestPtA += vertices[i].l * vertices[i].wA;
			closestPtB += vertices[i].l * vertices[i].wB;
		}
		
		output.distance = dir.magnitude();
		output.iterations = itr;
		
		simplex.WriteCache(cache);
		
		if (input.useRadii)
		{
			float rA = proxyA.m_radius;
			float rB = proxyB.m_radius;

			if (output.distance > rA + rB && output.distance > tolerance)
			{
				// Shapes are still no overlapped.
				// Move the witness points to the outer surface.
				output.distance -= rA + rB;
				vec2 normal = output.pointB - output.pointA;
				normal.normalize();
				output.pointA += rA * normal;
				output.pointB -= rB * normal;
			}
			else
			{
				// Shapes are overlapped when radii are considered.
				// Move the witness points to the middle.
				vec2 p = 0.5f * (output.pointA + output.pointB);
				output.pointA = p;
				output.pointB = p;
				output.distance = 0.0f;
			}
		}
		
	}
	
}