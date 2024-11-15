
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
	std::vector<float> S1D(Simplex& simplex, boolean debugSpit = false)
	{
		const vec2& s1 = simplex[0].w;
		const vec2& s2 = simplex[1].w;
		vec2 t = s2 - s1;
		
		// orthogonal projection of the origin onto the infinite line s1s2
		// (Paper was wrong with the formula here omfg)
		vec2 p0 = s1 + (-s1.dot(t) / t.dot(t)) * t;

		if (debugSpit)
		{
			std::cout << "\t\t S1D: " << std::endl;
			std::cout << "\t\t s1: " << s1 << std::endl;
			std::cout << "\t\t s2: " << s2 << std::endl;
			std::cout << "\t\t t: " << t << std::endl;
			std::cout << "\t\t p0" << p0 << std::endl;
		}

		// Calculate barycentric coordinates for s1 and s2 based on p0
		// Reduce to the dimension with the largest absolute value
		float mu_max = s1.x - s2.x;
		int I = 0;
		if (std::abs(s1.y - s2.y) > std::abs(mu_max)) {
			mu_max = s1.y - s2.y;
			I = 1; // Track which component is most influential
		}

		if (debugSpit)
		{
			std::cout << "\t\t mu_max: " << mu_max << std::endl;
			std::cout << "\t\t I: " << I << std::endl;
			if (I == 0)
				std::cout << "\t\t x-axis deemed influential" << std::endl;
			else
				std::cout << "\t\t y-axis deemed influential" << std::endl;
		}

		// Calculate signed area relative to the coordinate with the largest difference
		float C[2];
		C[0] = -(s2[I] - p0[I]);
		C[1] = s1[I] - p0[I];

		if (debugSpit)
		{
			std::cout << "\t\t Calculated signed-area: " << std::endl;
			std::cout << "\t\t C[0]: " << C[0] << std::endl;
			std::cout << "\t\t C[1]: " << C[1] << std::endl;
		}

		// Determine whether to keep the full simplex or reduce it (compare signs algo)
		bool allSignsMatch = compareSigns(C[0], mu_max) && compareSigns(C[1], mu_max);

		if (allSignsMatch)
		{
			if (debugSpit)
			{
				std::cout << "\t\t All signs matched, will not modify simplex" << std::endl;
				std::cout << "\t\t Barycentric[0]: " << C[0] / mu_max << std::endl;
				std::cout << "\t\t Barycentric[1]: " << C[1] / mu_max << std::endl;
			}
			return { C[0] / mu_max, C[1] / mu_max };
		}
		else
		{
			if (debugSpit)
			{
				std::cout << "\t\t Signs unmatched, reducing simplex!" << std::endl;
			}
			simplex = { simplex[0] };
		}
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

		if (debugSpit)
		{
			std::cout << "\t S2D: " << std::endl;
			std::cout << "\t s1: " << s1 << std::endl;
			std::cout << "\t s2: " << s2 << std::endl;
			std::cout << "\t s3: " << s3 << std::endl;
		}

		// no need to calculate p0 (projection of origin onto plane), as the origin will lie in the same plane as these points
		
		// Find signed area
		// Normally we reduce to the dimension with the largest absolute value, but since
		// we are already in 2D, our vectors don't have a 3rd component to reduce,
		// so we just leave them alone (in 3D, we would have to remove the coordinate to project everything into 2D)
		// the signed area of the triangle remains the same in 2D
		float mu_max =	/*s2.x * s3.y +
						s1.x * s2.y +
						s3.x * s1.y -
						s2.x * s1.y -
						s3.x * s2.y -
						s1.x * s3.y;*/
						s1.x * (s2.y - s3.y) +
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
		
		if (debugSpit)
		{
			std::cout << "\t C1: " << C1 << std::endl;
			std::cout << "\t C2: " << C2 << std::endl;
			std::cout << "\t C3: " << C3 << std::endl;
			std::cout << "\t mu_max: " << mu_max << std::endl;
		}

		bool cmp1 = compareSigns(mu_max, C1),
			cmp2 = compareSigns(mu_max, C2),
			cmp3 = compareSigns(mu_max, C3);
		
		if (cmp1 && cmp2 && cmp3)
		{
			if (debugSpit)
			{
				std::cout << "\t Simplex encloses origin, returning barycentric; " << std::endl;
			}

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
		if (debugSpit)
		{
			std::cout << "\tSimplex does not enclose origin, determine closest edge" << std::endl;
		}
		if (!cmp2)
		{
			// s2 appears to be non-contributing, so we check the reduced simplex { s1. s3 }
			if (debugSpit)
			{
				std::cout << "\t Reducing simplex to { " << simplex[0] << "," << simplex[2] << std::endl;
			}
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
			if (debugSpit)
			{
				std::cout << "\t Reducing simplex to { " << simplex[0] << "," << simplex[1] << std::endl;
			}
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
			if (debugSpit)
			{
				std::cout << "\t Reducing simplex to { " << simplex[1] << "," << simplex[2] << std::endl;
			}
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

		if (debugSpit)
		{
			std::cout << "\t In Distance Subalgorithm:" << std::endl;
			std::cout << "\t Dimensionality: " << dim << std::endl;
		}
		
		switch (dim)
		{
		case 1:
			if (debugSpit)
			{
				std::cout << "\t Entering S1D..." << std::endl;
			}
			return S1D(outSimplex, debugSpit);
		case 2:
			if (debugSpit)
			{
				std::cout << "\t Entering S2D..." << std::endl;
			}
			return S2D(outSimplex, debugSpit);
		default:
			if (debugSpit)
			{
				std::cout << "\t Defaulted..." << std::endl;
			}
			return { 1.0f }; // leave the simplex as is
		}
	}
	#pragma endregion

	GJKresult GJK(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex, boolean debugSpit)
	{
		if (debugSpit)
		{
			std::cout << "\n===========================================================" << std::endl;
			std::cout << " -- GJK DEBUG INFO --" << std::endl;

			std::cout << " INPUTS " << std::endl;
			std::cout << " Primary Object: " << std::endl;
			std::cout << inPrimary << std::endl;
			std::cout << " Target Object: " << std::endl;
			std::cout << inTarget << std::endl;
			std::cout << "\n -- Main algorithm begins -- " << std::endl;
		}
		std::vector<Mvert> previousPoints;
		GJKresult result;
		vec2 dir = inPrimary.position - inTarget.position;
		if (outSimplex.size())
			dir = outSimplex[0].w;

		if (debugSpit)
		{
			std::cout << "Intial direction: " << dir;
		}

		for (int itr = 0; itr < commons::GJK_ITERATIONS; itr++)
		{
			if (debugSpit)
			{
				std::cout << std::endl;
				std::cout << " -- ITERATION " << itr << " --" << std::endl;
				std::cout << " Current Direction: " << dir << std::endl;
				std::cout << " Negated Direction: " << -dir << std::endl;
			}

			float dirm = dir.sqrMagnitude();
			Mvert w = GetSupportVertex(inPrimary, inTarget, -dir);

			if (debugSpit)
			{
				std::cout << "dir sqrMag: " << dirm << std::endl;
				std::cout << "New support point found: " << w << std::endl;
			}
			
			if (outSimplex.isDupe(w))
			{
				if (debugSpit)
				{
					std::cout << " ** FOUND DUPLICATE POINT IN SIMPLEX, TERMINATING GJK LOOP ** " << std::endl;
				}
				break;
			}
			if (debugSpit)
			{
				printf("dir.sqrMag: %.10f\n", dirm);
				printf("dir.dot(w.w): %.10f\n", dir.dot(w.w));
				std::cout << "dirm * epsilon sqr = " << (dirm * commons::LEPSILON * commons::LEPSILON) << std::endl;
				std::cout << "dirm - dir.dot(w.w) = " << (dirm - dir.dot(w.w)) << std::endl;
			}
			if ((dirm - dir.dot(w.w)) <= (dirm * commons::LEPSILON * commons::LEPSILON)) // Termination condition A
			{
				if (debugSpit)
				{
					std::cout << " ** TERMINATION CONDITION A MET, TERMINATING GJK LOOP ** " << std::endl;
				}
				break;
			}
			previousPoints.push_back(w);
			outSimplex.push_front(w);
			if (debugSpit)
			{
				std::cout << " Simplex updated to now be: ";
				std::cout << outSimplex << std::endl;

				std::cout << "\n------------ start of subalgorithm ------------ " << std::endl;
			}
			
			std::vector<float> lambdas = signedVolumeDistanceSubalgorithm(outSimplex, debugSpit);

			if (debugSpit)
			{
				std::cout << "------------ end of subalgorithm ------------ " << std::endl;
				std::cout << "Lambdas calculated: ";
				for (float l : lambdas)
				{
					std::cout << l << " , ";
				}
				std::cout << std::endl;
				std::cout << " Simplex updated after dist algo to be: ";
				std::cout << outSimplex << std::endl;
			}

			//We determine the closest points on each shape via the barycentric coordinates
			dir = vec2::zero;
			result.z1 = vec2::zero;
			result.z2 = vec2::zero;
			for (int l = 0; l < lambdas.size(); l++) // the size of the simplex should always be the size of the lambdas
			{
				result.z1 += lambdas[l] * outSimplex[l].a;
				result.z2 += lambdas[l] * outSimplex[l].b;
				dir += lambdas[l] * outSimplex[l].w;
				if (debugSpit)
				{
					printf("lambda[l]: %.4f\n", lambdas[l]);
					std::cout << "Working on vertex: " << outSimplex[l] << std::endl;
					std::cout << " zA: " << result.z1 << std::endl;
					std::cout << " zB: " << result.z2 << std::endl;
				}
			}

			if (debugSpit)
			{
				std::cout << "New direction: " << dir << std::endl;
				std::cout << "Closest point on A: " << result.z1 << std::endl;
				std::cout << "Closest point on B: " << result.z2 << std::endl;
			}
				
			if (outSimplex.size() >= 3) // Termination condition B
			{
				if (debugSpit)
				{
					std::cout << " ** SIMPLEX SIZE AT MAX 3, TERMINATING GJK LOOP ** " << std::endl;
				}
				break;
			}

			float max_norm = 1.0f;
			for (const auto& m : outSimplex)
			{
				float norm = m.w.sqrMagnitude();
				max_norm = (max_norm > norm) ? max_norm : norm;
			}

			if (debugSpit)
			{
				std::cout << "max_norm: " << max_norm << std::endl;
			}
			
			if (dir.sqrMagnitude() < (commons::HEPSILON * max_norm)) // Termination condition B
			{
				if (debugSpit)
				{
					std::cout << " ** TERMINATION CONDITION B MET, TERMINATING GJK LOOP ** " << std::endl;
				}
				break;
			}
		}
		

		result.distance = dir.magnitude();
		if (debugSpit)
		{
			std::cout << "EXITED GJK ITERATIVE LOOP!" << std::endl;
			std::cout << "FINAL CALCULATED DISTANCE = "; printf("%.10f\n", result.distance);
			std::cout << "===========================================================" << std::endl;
		}
		return result;
	}

#pragma region gjk mu
	//std::array<float, 3> SignedVolumeDSA(Simplex& simplex);
	//std::array<float, 3> S1D(Simplex& simplex);
	//std::array<float, 3> S2D(Simplex& simplex);

	//float GJK(const GJKobject& inPrimary, const GJKobject& inTarget, CollisionStatus& status)
	//{
	//	Simplex simplex;
	//	vec2 v = inPrimary.position - inTarget.position;
	//	float eps = status.tolerance * status.tolerance;
	//	int get_contacts = status.flags & 0x1;
	//	int get_dist = status.flags & 0x2;
	//	
	//	int itr = 0, max_itr = status.max_iterations;
	//	for (; itr < max_itr; itr++)
	//	{
	//		Mvert w = GetSupportVertex(inPrimary, inTarget, v);
	//		
	//		vec2 diff = w.w - v;
	//		if (2 * v.dot(diff) < eps)
	//			break;
	//		
	//		if (!get_dist && v.dot(w.w) > 0)
	//			return FLT_MAX;
	//		
	//		simplex.push_front(w);
	//		std::array<float, 3> lambdas = SignedVolumeDSA(simplex);
	//		
	//		vec2 v_s = vec2::zero;
	//		status.z1 = vec2::zero;
	//		status.z2 = vec2::zero;
	//		for (int l = 0; l < lambdas.size(); l++)
	//		{
	//			status.z1 += lambdas[l] * simplex[l].a;
	//			status.z2 += lambdas[l] * simplex[l].b;
	//			v_s += lambdas[l] * simplex[l].w;
	//		}

	//		if ((v_s - v).sqrMagnitude() <= eps)
	//			break;
	//		
	//		v = v_s;
	//		
	//		if (simplex.size() >= 3)
	//			break;
	//		
	//	}
	//	status.gjk_iterations = itr;
	//	status.simplex = simplex;
	//	return v.magnitude();
	//}
	//
	//std::array<float, 3> SignedVolumeDSA(Simplex& simplex)
	//{
	//	int dim = simplex.size() - 1;
	//	switch (dim)
	//	{
	//	case 2:
	//		return S2D(simplex);
	//	case 1:
	//		return S1D(simplex);
	//	case 0:
	//	default:
	//		return { 1.0f, 0.0f, 0.0f };
	//	}
	//}

	//std::array<float, 3> S1D(Simplex& simplex)
	//{
	//	const vec2& s1 = simplex[0].w;
	//	const vec2& s2 = simplex[1].w;
	//	vec2 t = s2 - s1;

	//	// orthogonal projection of the origin onto the infinite line s1s2
	//	// (Paper was wrong with the formula here omfg)
	//	vec2 p0 = s1 + (-s1.dot(t) / t.dot(t)) * t;
	//	
	//	// Calculate barycentric coordinates for s1 and s2 based on p0
	//	// Reduce to the dimension with the largest absolute value
	//	float mu_max = s1.x - s2.x;
	//	int I = 0;
	//	if (std::abs(s1.y - s2.y) >= std::abs(mu_max)) {
	//		mu_max = s1.y - s2.y;
	//		I = 1; // Track which component is most influential
	//	}

	//	float C1 = p0[I] - s2[I];
	//	float C2 = s1[I] - p0[I];

	//	if (compareSigns(mu_max, C1) && compareSigns(mu_max, C2))
	//	{
	//		return {
	//			C1 / mu_max,
	//			C2 / mu_max,
	//			0.0f
	//		};
	//	}
	//	else
	//	{
	//		simplex = { simplex[0] };
	//		return {
	//			1.0f,
	//			0.0f,
	//			0.0f
	//		};
	//	}
	//}

	//float weightedDistance(const std::array<float, 3>& coeff, const Simplex& simplex)
	//{
	//	vec2 weightedSum = vec2::zero; // Initialize the weighted sum as a 2D vector
	//	for (size_t i = 0; i < simplex.size(); ++i) {
	//		weightedSum += coeff[i] * simplex[i].w; // Accumulate weighted points
	//	}
	//	return weightedSum.magnitude(); // Return the magnitude of the weighted sum
	//}

	//std::array<float, 3> S2D(Simplex& simplex)
	//{
	//	const vec2& s1 = simplex[0].w;
	//	const vec2& s2 = simplex[1].w;
	//	const vec2& s3 = simplex[2].w;

	//	// no need to calculate p0 (projection of origin onto plane), as the origin will lie in the same plane as these points

	//	// Find signed area
	//	// Normally we reduce to the dimension with the largest absolute value, but since
	//	// we are already in 2D, our vectors don't have a 3rd component to reduce,
	//	// so we just leave them alone (in 3D, we would have to remove the coordinate to project everything into 2D)
	//	// the signed area of the triangle remains the same in 2D
	//	float mu_max = s2.x * s3.y +
	//		s1.x * s2.y +
	//		s3.x * s1.y -
	//		s2.x * s1.y -
	//		s3.x * s2.y -
	//		s1.x * s3.y;

	//	// Corresponds to the signed area of 2-simplex: (p0, s2, s3)
	//	float C1 = s2.cross(s3);
	//	// Corresponds to the signed area of 2-simplex: (p0, s1, s3)
	//	float C2 = s1.cross(s3);
	//	// Corresponds to the signed area of 2-simplex: (p0, s1, s2)
	//	float C3 = s1.cross(s2);

	//	bool cmp1 = compareSigns(mu_max, C1),
	//		 cmp2 = compareSigns(mu_max, C2),
	//		 cmp3 = compareSigns(mu_max, C3);

	//	if (cmp1 && cmp2 && cmp3)
	//	{
	//		// encloses origin in 2D
	//		return {
	//			C1 / mu_max,
	//			C2 / mu_max,
	//			C3 / mu_max
	//		};
	//	}
	//	
	//	float d = FLT_MAX; // minimum distance to origin
	//	std::array<float, 3> l = { 1.0f, 0.0f, 0.0f }; // default to s1
	//	if (!cmp2)
	//	{
	//		// s2 appears to be non-contributing, so we check the reduced simplex { s1. s3 }
	//		simplex = { simplex[0], simplex[2] };
	//		auto ls = S1D(simplex);
	//		float ds = weightedDistance(ls, simplex);
	//		if (d < ds)
	//		{
	//			l = ls;
	//			d = ds;
	//		}
	//	}
	//	
	//	if (!cmp3)
	//	{
	//		// s3 appears to be non-contributing, so we check the reduced simplex { s1. s2 }
	//		simplex = { simplex[0], simplex[1] };
	//		auto ls = S1D(simplex);
	//		float ds = weightedDistance(ls, simplex);
	//		if (d < ds)
	//		{
	//			l = ls;
	//			d = ds;
	//		}
	//	}
	//	
	//	if (!cmp1)
	//	{
	//		// s1 appears to be non-contributing, so we check the reduced simplex { s1. s2 }
	//		// Unlikely to reach here, as s1 is meant to be the latest point towards the origin
	//		simplex = { simplex[1], simplex[2] };
	//		auto ls = S1D(simplex);
	//		float ds = weightedDistance(ls, simplex);
	//		if (d < ds)
	//		{
	//			l = ls;
	//			d = ds;
	//		}
	//	}

	//	return l;
	//}
#pragma endregion

	GJKresult EPA(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex)
	{

	}
}