
#include "pch.h"
#include "gjk.h"
#include "cprocessing.h" //!!TO REMOVE!!

namespace chiori
{
	bool floatEqual(float a, float b) { return std::abs(a - b) < commons::HEPSILON; }
	
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
	std::vector<float> S1D(Simplex& outSimplex)
	{
		const vec2& s1 = outSimplex[0].w;
		const vec2& s2 = outSimplex[1].w;
		vec2 t = s1 - s2;
		
		// orthogonal projection of the origin onto the infinite line s1s2
		vec2 p0 = s2 + (s2.dot(t) / t.dot(t)) * t;

		// Calculate barycentric coordinates for s1 and s2 based on p0
		// Reduce to the dimension with the largest absolute value
		float mu_max = s1.x - s2.x;
		int I = 0;
		if (std::abs(s1.y - s2.y) > std::abs(mu_max)) {
			mu_max = s1.y - s2.y;
			I = 1; // Track which component is most influential
		}

		// Calculate barycentric coordinates relative to the coordinate with the largest difference
		float C[2];
		for (int j = 0; j < 2; ++j) {
			// We calculate the coordinate-wise contribution for barycentric coordinates
			C[j] = (j == 0 ? 1.0f : -1.0f) * (s1[I] - p0[I]);
		}

		// Determine whether to keep the full simplex or reduce it (compare signs algo)
		bool allSignsMatch = std::all_of(std::begin(C), std::end(C), [mu_max](float cj) {
			return (mu_max > 0 && cj > 0) || (mu_max < 0 && cj < 0);
			});
		
		if (allSignsMatch)
		{
			return { C[0] / mu_max, C[1] / mu_max };
		}
		else
			outSimplex = { outSimplex[0] };

		return { 1.0f };
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
		const vec2& s1 = outSimplex[0].w;
		const vec2& s2 = outSimplex[1].w;
		const vec2& s3 = outSimplex[2].w;

		// no need to calculate p0 (projection of origin onto plane), as the origin will lie in the same plane as these points
		
		// Find signed area
		// Normally we reduce to the dimension with the largest absolute value, but since
		// we are already in 2D, our vectors don't have a 3rd component to reduce,
		// so we just leave them alone (in 3D, we would have to remove the coordinate to project everything into 2D)
		float mu_max = s2.x * s3.y + s1.x * s2.y + s3.x * s1.y - s2.x * s1.y
						- s3.x * s2.y - s1.x * s3.y;
		float mu_other = s2.y * s3.x + s1.y * s2.x + s3.y * s1.x - s2.y * s1.x
						- s3.y * s2.x - s1.y * s3.x;
		
		if (std::abs(mu_other) > std::abs(mu_max))
			mu_max = mu_other;

		// Calculate barycentric coordinates for s1, s2, and s3
		// In the paper it uses a determinant calculation, which we can simplify in 2D
		// to a simple 2D cross product. Hell yeah.
		float C[3];
		for (int j = 0; j < 3; j++)
		{
			C[j] = (j == 0 ? 1.0f : -1.0f) * (outSimplex[j].w.cross(outSimplex[(j + 1) % 3].w));
		}
		
		// Determine whether to keep the full simplex or reduce it (compare signs algo)
		bool allSignsMatch = std::all_of(std::begin(C), std::end(C), [mu_max](float cj) {
			return (mu_max > 0 && cj > 0) || (mu_max < 0 && cj < 0);
			});	

		if (allSignsMatch)
		{
			return {
				C[0] / mu_max,
				C[1] / mu_max,
				C[2] / mu_max
			};
		}
		
		float d = FLT_MAX;
		std::vector<float> l;
		for (int k = 0; k < 2; k++)
		{
			if (compareSigns(mu_max, -C[k + 1]))
			{
				Simplex reducedSimplex;
				reducedSimplex = { outSimplex[0], outSimplex[2 - k] };
				std::vector<float> ls = S1D(reducedSimplex);
				float ds = computeDStar(reducedSimplex, l);
				if (ds < d)
				{
					outSimplex = reducedSimplex;
					l = ls;
					d = ds;
				}
			}
		}

		return l;
	}
	
	//std::vector<float> S1D_T(Simplex& outSimplex) // 1D Simplex case
	//{
	//	std::vector<float> result;
	//	
	//	const vec2& s1 = outSimplex[0].w;
	//	const vec2& s2 = outSimplex[1].w;
	//	vec2 t = s1 - s2;
	//	vec2 p0 = (s2.dot(t)) / (t.dot(t)) * t + s2; // orthogonal projection of the origin onto the infinite line s1s2
	//	float mu_max = 0.0f; int I = -1;
	//	// Step through coordinates of s1 and s2 to find the maximum |mu| (component wise difference)
	//	for (int i = 0; i < 2; ++i) {
	//		float mu = s1[i] - s2[i];
	//		if (std::abs(mu) > std::abs(mu_max)) {
	//			mu_max = mu;
	//			I = i;
	//		}
	//	} // we do this to reduce the problem to just one dimension with the greatest difference (x or y)
	//	
	//	// calculate the barycentric coordinates of p0 relative to s1 and s2
	//	int k = 2;
	//	std::array<float, 2> C;
	//	for (int j = 0; j < 2; ++j) {
	//		C[j] = (j % 2 == 0 ? 1.0f : -1.0f) * (s1[I] - p0[I]);
	//		k = j;
	//	}
	//	
	//	// we check if the barycentric coordinates have the same sign as mu_max
	//	// if it does, we know p0 lies between s1 and s2
	//	// if it doesn't, it must lie outside, with the closest point to the origin being s1
	//	// as the first vertex in the simplex is ALWAYS the latest updated
	//	bool allSignsMatch = true;
	//	for (int j = 0; j < 2; ++j) {
	//		if (!compareSigns(mu_max, C[j])) {
	//			allSignsMatch = false;
	//			break;
	//		}
	//	}
	//	if (allSignsMatch) {
	//		// update barycentric coordinates
	//		result = { (C[0] / mu_max), (C[1] / mu_max) };
	//		// we keep the simplex untouched
	//	}
	//	else {
	//		result = { 1.0f }; // since only s1 contributes, the barycentric coordinates is just 1
	//		outSimplex = { outSimplex[0] }; // we keep only s1
	//	}
	//	return result;
	//}
	//std::vector<float> S2D_T(Simplex& outSimplex)
	//{
	//	// This is modified from the paper as it attempts to determine the closest
	//	// point on the plane the origin is to, however, in 2D, the origin ALREADY
	//	// lies on the same plane (hence the closest point would be itself).
	//	// So, we can easily remove a segment of calulations and simplify by
	//	// treating p0 as 0,0 in the pseudocode
	//	std::vector<float> result;
	//	const vec2& s1 = outSimplex[0].w;
	//	const vec2& s2 = outSimplex[1].w;
	//	const vec2& s3 = outSimplex[2].w;
	//	// We skip the `p0` projection as it’s just the origin (0,0) in 2D
	//	float mu_max = 0.0f;
	//	int J = -1;
	//	// Calculate mu for x and y components (we dont put this in a loop to avoid the headache)
	//	float mu_x = s2.x * s3.y + s3.x * s1.y + s1.x * s2.y - s3.x * s2.y - s2.x * s1.y - s1.x * s3.y;
	//	float mu_y = s2.y * s3.x + s3.y * s1.x + s1.y * s2.x - s3.y * s2.x - s2.y * s1.x - s1.y * s3.x;
	//	// Find the maximum mu component
	//	if (std::abs(mu_x) > std::abs(mu_max)) {
	//		mu_max = mu_x;
	//		J = 0;  // Indicates x component
	//	}
	//	if (std::abs(mu_y) > std::abs(mu_max)) {
	//		mu_max = mu_y;
	//		J = 1;  // Indicates y component
	//	}
	//	
	//	int k = 2;
	//	int l = 3;
	//	std::array<float, 3> C;
	//	for (int j = 2; j <= 3; ++j)
	//	{
	//		const vec2& s_k = outSimplex[k - 1].w; // Convert from 1-based to 0-based indexing
	//		const vec2& s_l = outSimplex[l - 1].w; // Same conversion for l
	//		int sign = (j % 2 == 0) ? 1 : -1;
	//		C[j-1] = sign * (s_k[0] * s_l[1] - s_l[0] * s_k[1]); // simplified when removing p0
	//		k = l; l = j;
	//	}
	//	// we check if the barycentric coordinates have the same sign as mu_max
	//	bool allSignsMatch = true;
	//	for (int j = 0; j < C.size(); ++j) {
	//		if (!compareSigns(mu_max, C[j])) {
	//			allSignsMatch = false;
	//			break;
	//		}
	//	}
	//	if (allSignsMatch) {
	//		// update barycentric coordinates
	//		result = {
	//			C[0] / mu_max,
	//			C[1] / mu_max,
	//			C[2] / mu_max
	//		};
	//		// we keep the simplex untouched
	//	}
	//	else
	//	{
	//		float d = FLT_MAX;
	//		// j = 2, exclude s2, exclude simplex[1]
	//		// j = 3, exclude s3, exclude simplex[2]
	//		for (int j = 2; j <= 3; ++j)
	//		{
	//			if (!compareSigns(mu_max, -C[j - 1]))
	//				continue;
	//			Simplex simplexStar;
	//			simplexStar = { outSimplex[0], outSimplex[j - 1]};
	//			std::vector<float> lambdaStar = S1D(simplexStar);
	//			
	//			float dstar = computeDStar(simplexStar, lambdaStar);
	//			if (dstar < d)
	//			{
	//				outSimplex = simplexStar;
	//				result = lambdaStar;
	//				d = dstar;
	//			}
	//		}
	//	}
	//	
	//	return result;
	//}

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
			return { 1.0f }; // leave the simplex as is
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
			Mvert w = GetSupportVertex(inPrimary, inTarget, -dir);
			if (outSimplex.isDupe(w)) // Termination condition A
				break;
			if ((dirm - dir.dot(w.w)) <= (dirm * commons::HEPSILON * commons::HEPSILON) ) // Termination condition A
				break;
			
			outSimplex.push_front(w);
			
			std::vector<float> lambdas = SignedVolumeDistanceSubalgorithm(outSimplex);

			//We determine the closest points on each shape via the barycentric coordinates
			dir = vec2::zero;
			for (int l = 0; l < lambdas.size(); l++) // the size of the simplex should always be the size of the lambdas
			{
				result.zA += lambdas[l] * outSimplex[l].a;
				result.zB += lambdas[l] * outSimplex[l].b;
				dir += lambdas[l] * outSimplex[l].w;
			}
				
			if (outSimplex.size() >= 3) // Termination condition B
				break;

			float max_norm = 1.0f;
			for (const auto& m : outSimplex)
			{
				float norm = m.w.sqrMagnitude();
				max_norm = (max_norm > norm) ? max_norm : norm;
			}
			
			if (dir.sqrMagnitude() < (commons::HEPSILON * max_norm)) // Termination condition B
				break;
		}
		result.distance = dir.magnitude();
		return result;
	}
	
}