
#include "pch.h"
#include "gjk.h"

namespace chiori
{
	vec2 GJKobject::getSupportPoint(const vec2& inDir) const
	{
		vec2 result = vertices[0];
		float maxDot = result.dot(inDir);

		for (int i = 1; i < vertices.size(); i++)
		{
			float dot = vertices[i].dot(inDir);
			if (dot > maxDot)
			{
				maxDot = dot;
				result = vertices[i];
			}
		}
		return result;
	}

	bool compareSigns(float a, float b)
	{
		return (a > 0 && b > 0) || (a < 0 && b < 0);
	}
	
	#pragma region Distance Subalgorithm
	std::vector<float> S1D(Simplex& outSimplex) // 1D Simplex case
	{
		std::vector<float> result;
		
		const vec2& s1 = outSimplex[0].w;
		const vec2& s2 = outSimplex[1].w;
		vec2 t = s1 - s2;
		vec2 p0 = (s2.dot(t)) / (t.dot(t)) * t + s2; // orthogonal projection of the origin onto the infinite line s1s2

		float mu_max = 0.0f; int I = -1;
		// Step through coordinates of s1 and s2 to find the maximum |mu| (component wise difference)
		for (int i = 0; i < 2; ++i) {
			float mu = s1[i] - s2[i];
			if (std::abs(mu) > std::abs(mu_max)) {
				mu_max = mu;
				I = i;
			}
		} // we do this to reduce the problem to just one dimension with the greatest difference (x or y)
		
		// calculate the barycentric coordinates of p0 relative to s1 and s2
		int k = 2;
		std::array<float, 2> C;
		for (int j = 0; j < 2; ++j) {
			C[j] = (j % 2 == 0 ? 1.0f : -1.0f) * (s1[I] - p0[I]);
			k = j;
		}
		
		// we check if the barycentric coordinates have the same sign as mu_max
		// if it does, we know p0 lies between s1 and s2
		// if it doesn't, it must lie outside, with the closest point to the origin being s1
		// as the first vertex in the simplex is ALWAYS the latest updated
		bool allSignsMatch = true;
		for (int j = 0; j < 2; ++j) {
			if (!compareSigns(mu_max, C[j])) {
				allSignsMatch = false;
				break;
			}
		}

		if (allSignsMatch) {
			// update barycentric coordinates
			result.push_back(C[0] / mu_max);
			result.push_back(C[1] / mu_max);
			// we keep the simplex untouched
		}
		else {
			result.push_back(1.0f); // since only s1 contributes, the barycentric coordinates is just 1
			outSimplex = { outSimplex[0] }; // we keep only s1
		}
		return result;
	}

	float computeDStar(const Simplex& inWstar, const std::vector<float>& inLambdastar)
	{
		vec2 weightedSum = { 0.0f, 0.0f }; // Initialize the weighted sum as a 2D vector
		for (size_t i = 0; i < inWstar.size(); ++i) {
			weightedSum += inLambdastar[i] * inWstar[i].w; // Accumulate weighted points
		}
		return weightedSum.magnitude(); // Return the magnitude of the weighted sum
	}

	std::vector<float> S2D(Simplex& outSimplex)
	{
		// This is modified from the paper as it attempts to determine the closest
		// point on the plane the origin is to, however, in 2D, the origin ALREADY
		// lies on the same plane (hence the closest point would be itself).
		// So, we can easily remove a segment of calulations and simplify by
		// treating p0 as 0,0 in the pseudocode

		std::vector<float> result;

		const vec2& s1 = outSimplex[0].w;
		const vec2& s2 = outSimplex[1].w;
		const vec2& s3 = outSimplex[2].w;

		// We skip the `p0` projection as it’s just the origin (0,0) in 2D
		float mu_max = 0.0f;
		int J = -1;

		// Calculate mu for x and y components (we dont put this in a loop to avoid the headache)
		float mu_x = s2.x * s3.y + s3.x * s1.y + s1.x * s2.y - s3.x * s2.y - s2.x * s1.y - s1.x * s3.y;
		float mu_y = s2.y * s3.x + s3.y * s1.x + s1.y * s2.x - s3.y * s2.x - s2.y * s1.x - s1.y * s3.x;
		// Find the maximum mu component
		if (std::abs(mu_x) > std::abs(mu_max)) {
			mu_max = mu_x;
			J = 0;  // Indicates x component
		}
		if (std::abs(mu_y) > std::abs(mu_max)) {
			mu_max = mu_y;
			J = 1;  // Indicates y component
		}
		
		int k = 2;
		int l = 3;
		std::array<float, 3> C;
		for (int j = 2; j <= 3; ++j)
		{
			const vec2& s_k = outSimplex[k - 1].w; // Convert from 1-based to 0-based indexing
			const vec2& s_l = outSimplex[l - 1].w; // Same conversion for l
			int sign = (j % 2 == 0) ? 1 : -1;
			C[j] = sign * (s_k[0] * s_l[1] - s_l[0] * s_k[1]); // simplified when removing p0
			k = l; l = j;
		}

		// we check if the barycentric coordinates have the same sign as mu_max
		bool allSignsMatch = true;
		for (int j = 0; j < C.size(); ++j) {
			if (!compareSigns(mu_max, C[j])) {
				allSignsMatch = false;
				break;
			}
		}

		if (allSignsMatch) {
			// update barycentric coordinates
			result.push_back(C[0] / mu_max);
			result.push_back(C[1] / mu_max);
			result.push_back(C[2] / mu_max);
			// we keep the simplex untouched
		}
		else
		{
			float d = FLT_MAX;
			// j = 2, exclude s2, exclude simplex[1]
			// j = 3, exclude s3, exclude simplex[2]
			for (int j = 2; j <= 3; ++j)
			{
				if (!compareSigns(mu_max, -C[j]))
					continue;

				Simplex simplexStar;
				simplexStar = { outSimplex[0], outSimplex[j - 1]};
				std::vector<float> lambdaStar = S1D(simplexStar);
				
				float dstar = computeDStar(simplexStar, lambdaStar);
				if (dstar < d)
				{
					outSimplex = simplexStar;
					result = lambdaStar;
					d = dstar;
				}
			}
		}
		
		return result;
	}

	std::vector<float> SignedVolumeDistanceSubalgorithm(Simplex& outSimplex)
	{
		int dim = outSimplex.size() - 1;
		switch (outSimplex.size())
		{
		case 1:
			return S1D(outSimplex);
		case 2:
			return S2D(outSimplex);
		default:
			std::vector<float> result{ 1.0f };
			return result;
		}
	}
	#pragma endregion

	GJKresult GJKExtended(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex)
	{
		GJKresult result;
		vec2 dir = (outSimplex.size()) ? outSimplex[0].w : (inPrimary.position - inTarget.position);

		for (int itr = 0; itr < commons::GJK_ITERATIONS; itr++)
		{
			float dirm = dir.sqrMagnitude();
			Mvert w = GetSupportVertex(inPrimary, inTarget, dir);
			if ((outSimplex.isDupe(w)) || 
				(dirm - dir.dot(w.w) <= (dirm * commons::HEPISLON))) // Termination condition A
				break;
			
			outSimplex.push_front(w);
			
			std::vector<float> lambdas = SignedVolumeDistanceSubalgorithm(outSimplex);

			//CALCULATE FRM LAMBDAS
			
		}
		
		return result;
	}
	
}