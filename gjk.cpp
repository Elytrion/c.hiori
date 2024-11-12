
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
	std::vector<float> S1D(Simplex& outSimplex, boolean debugSpit = false)
	{
		const vec2& s1 = outSimplex[0].w;
		const vec2& s2 = outSimplex[1].w;
		vec2 t = s2 - s1;
		
		// orthogonal projection of the origin onto the infinite line s1s2
		vec2 p0 = (s2.dot(t) / t.dot(t)) * t + s2;

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

		// Calculate barycentric coordinates relative to the coordinate with the largest difference
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
			outSimplex = { outSimplex[0] };
		}

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

	std::vector<float> S2D(Simplex& outSimplex, boolean debugSpit = false)
	{
		const vec2& s1 = outSimplex[0].w;
		const vec2& s2 = outSimplex[1].w;
		const vec2& s3 = outSimplex[2].w;

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
		float mu_max = s2.x * s3.y + s1.x * s2.y + s3.x * s1.y - s2.x * s1.y
						- s3.x * s2.y - s1.x * s3.y;
		float mu_other = s2.y * s3.x + s1.y * s2.x + s3.y * s1.x - s2.y * s1.x
						- s3.y * s2.x - s1.y * s3.x;
		
		if (std::abs(mu_other) > std::abs(mu_max))
			mu_max = mu_other;

		// Calculate barycentric coordinates for s1, s2, and s3
		// In the paper it uses a determinant calculation, which we can simplify in 2D
		// to a simple 2D cross product. Hell yeah.
		float C[2]; // we dont have to explicitly calculate the 3rd barycentric coordinate since sum of all them will = 1
		C[0] = s2.cross(s3);
		C[1] = s3.cross(s1);

		if (debugSpit)
		{
			std::cout << "\t mu_max: " << mu_max << std::endl;
			std::cout << "\t Calculated signed-area: " << std::endl;
			std::cout << "\t C[0]: " << C[0] << std::endl;
			std::cout << "\t C[1]: " << C[1] << std::endl;
		}
		
		// Determine whether to keep the full simplex or reduce it (compare signs algo)
		bool allSignsMatch = compareSigns(C[0], mu_max) && compareSigns(C[1], mu_max);

		if (allSignsMatch)
		{
			float l2 = C[0] / mu_max;
			float l3 = C[1] / mu_max;
			if (debugSpit)
			{
				std::cout << "\t All signs matched, will not modify simplex" << std::endl;
				std::cout << "\t Barycentric[0]: " << 1 - l2 - l3 << std::endl;
				std::cout << "\t Barycentric[1]: " << l2 << std::endl;
				std::cout << "\t Barycentric[2]: " << l3 << std::endl;
			}
			return {
				1 - l2 - l3,
				l2,
				l3
			};
		}
		
		if (debugSpit)
		{
			std::cout << "\t Signs unmatched, reducing simplex!" << std::endl;
		}
		float d = FLT_MAX;
		std::vector<float> l;
		for (int k = 0; k < 2; k++)
		{
			if (compareSigns(mu_max, -C[k]))
			{
				if (debugSpit)
				{
					std::cout << "\t Comparing mu_max and -C[" << k << "]: same sign!" << std::endl;
				}
				
				Simplex reducedSimplex;
				reducedSimplex = { outSimplex[0], outSimplex[2 - k] };
				if (debugSpit)
				{
					std::cout << "\t Reduced simplex with points [0] & [" << 2 - k << "]" << std::endl;
					std::cout << reducedSimplex << std::endl;
				}
				std::vector<float> ls = S1D(reducedSimplex);
				float ds = computeDStar(reducedSimplex, l);
				if (debugSpit)
				{
					std::cout << "\t Calcuated values from internal S1D: " << std::endl;
					std::cout << "\t Lambdas: ";
					for (float ll : ls)
					{
						std::cout << ll << " , ";
					}
					std::cout << std::endl;
					std::cout << "\t D*: " << ds << std::endl;
				}
				if (ds < d)
				{
					if (debugSpit)
					{
						std::cout << "\t Found ds < d" << std::endl;
						std::cout << "\t prev d " << d << std::endl;
						std::cout << "\t new d " << ds << std::endl;
					}

					outSimplex = reducedSimplex;
					l = ls;
					d = ds;
				}
			}
		}

		return l;
	}

	std::vector<float> SignedVolumeDistanceSubalgorithm(Simplex& outSimplex, boolean debugSpit = false)
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

	GJKresult GJKExtended(const GJKobject& inPrimary, const GJKobject& inTarget, Simplex& outSimplex, boolean debugSpit)
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
			if ((dirm - dir.dot(w.w)) <= (dirm * commons::HEPSILON * commons::HEPSILON)) // Termination condition A
			{
				if (debugSpit)
				{
					std::cout << " ** TERMINATION CONDITION A MET, TERMINATING GJK LOOP ** " << std::endl;
				}
				break;
			}
			
			outSimplex.push_front(w);
			if (debugSpit)
			{
				std::cout << " Simplex updated to now be: ";
				std::cout << outSimplex << std::endl;

				std::cout << "\n------------ start of subalgorithm ------------ " << std::endl;
			}
			
			std::vector<float> lambdas = SignedVolumeDistanceSubalgorithm(outSimplex, debugSpit);

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
			result.zA = vec2::zero;
			result.zB = vec2::zero;
			for (int l = 0; l < lambdas.size(); l++) // the size of the simplex should always be the size of the lambdas
			{
				result.zA += lambdas[l] * outSimplex[l].a;
				result.zB += lambdas[l] * outSimplex[l].b;
				dir += lambdas[l] * outSimplex[l].w;
			}

			if (debugSpit)
			{
				std::cout << "New direction: " << dir << std::endl;
				std::cout << "Closest point on A: " << result.zA << std::endl;
				std::cout << "Closest point on B: " << result.zB << std::endl;
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
			std::cout << "FINAL CALCULATED DISTANCE = " << result.distance << std::endl;
			std::cout << "===========================================================" << std::endl;
		}
		return result;
	}

	
}