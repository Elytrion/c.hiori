#include "pch.h"
#include "fracture.h"
#include <numeric>

namespace chiori
{
	float GetPolygonArea(cFragment& inPolygon)
	{
		auto& vertices = inPolygon.vertices;
		int n = vertices.size();
		if (n < 3) {
			return 0.0f; // A full polygon must have at least 3 vertices
		}
		float area = 0.0f;

		for (int i = 0; i < n; ++i) {
			const cVec2& current = vertices[i];
			const cVec2& next = vertices[(i + 1) % n]; // Wrap around to the first vertex
			area += (current.x * next.y - current.y * next.x);
		}
		return std::abs(area) / 2.0f;
	}

	static float computeRf(float density, const cFractureMaterial& matConfig)
	{
		float numerator = (matConfig.toughness * matConfig.elasticity);
		float denom = (density * matConfig.brittleness);
		cassert(denom > 0.0f);
		return matConfig.k * (numerator / denom);
	}

	static void computeStrainEnergyField(
		cFractureProxy& proxy, const cFractureMaterial& materialConfig,
		const cFractureImpact& impactConfig, float fractureRange, float density)
	{
		// Compute W(x) for each vertex
		for (int i = 0; i < proxy.count; ++i)
		{
			cVec2& vertex = proxy.fragment.vertices[i];
			float totalWeight = 0.0f;
			
			// Process each contact point
			for (int j = 0; j < impactConfig.contactCount; ++j)
			{
				const cVec2& contactPoint = impactConfig.localContacts[j];
				float impulseForce = impactConfig.impluseForces[j];
				
				float dist = distance(vertex, contactPoint);
				
				// If the vertex is outside the fracture range, skip (its W(x) is 0, no strain)
				if (dist > fractureRange)
					continue;
				
				float W = impulseForce * std::exp(-dist / fractureRange);
				float anisotropyFactor = 1.0f;
				if (materialConfig.anisotropyFactor > 0.0f) {
					cVec2 anisotropyDirection = materialConfig.anisotropy.normalized();
					cVec2 impactDirection = (vertex - contactPoint).normalized();
					float cosTheta = anisotropyDirection.dot(impactDirection);
					anisotropyFactor = 1 + materialConfig.anisotropyFactor * cosTheta * cosTheta;
				}

				// Apply material modifiers
				W /= materialConfig.toughness;
				W /= density; // Adjust for density
				W /= materialConfig.brittleness;
				W *= anisotropyFactor;

				totalWeight += W;
			}
			// Store the computed weight for the vertex
			proxy.weights[i] = totalWeight;
		}

		// Step 3: Normalize weights
		float totalWeight = std::accumulate(proxy.weights.begin(), proxy.weights.end(), 0.0f);
		if (totalWeight > 0.0f)
		{
			for (float& weight : proxy.weights) {
				weight /= totalWeight;
			}
		}
	}

	static int computeCentroidCount(float fractureRange, float objectArea, const cFractureMaterial& materialConfig)
	{
		// Step 1: Compute the fracture area (A_fracture)
		float fractureArea = PI * fractureRange * fractureRange;

		// Step 2: Compute the coverage ratio
		float coverageRatio = fractureArea / objectArea;

		// Step 3: Scale the number of centroids based on the coverage ratio
		int numCentroids = static_cast<int>(
			materialConfig.minPoints +
			(coverageRatio * (materialConfig.maxPoints - materialConfig.minPoints))
			);

		// Step 4: Clamp the result within the allowed range
		numCentroids = c_max(materialConfig.minPoints, c_min(materialConfig.maxPoints, numCentroids));

		return numCentroids;
	}

	static std::vector<cVec2> sampleCentroids(const cFractureProxy& proxy, const std::vector<float>& cdf, int numCentroids)
	{
		std::vector<cVec2> centroids;
		centroids.reserve(numCentroids);

		// Seeded random number generator using the proxy ID
		std::mt19937 rng(proxy.id); // Deterministic RNG
		std::uniform_real_distribution<float> dist(0.0f, 1.0f); // Random values in [0, 1]

		for (int i = 0; i < numCentroids; ++i) {
			float r = dist(rng); // Generate a random number

			// Find the index corresponding to r in the CDF using binary search
			auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
			size_t index = std::distance(cdf.begin(), it);

			// Add the corresponding vertex position as a centroid
			centroids.push_back(proxy.fragment.vertices[index]);
		}
		
		return centroids;
	}

	/// <summary>
	/// Fractures a given polygon detailed in cFractureProxy into smaller polygons based on the material and impact configs
	/// </summary>
	/// <param name="proxy">- Polygon to be fractured</param>
	/// <param name="materialConfig">- Material properties of the polygon</param>
	/// <param name="impactConfig">- Collision impulses applied to the polygon</param>
	/// <returns>A vector of fragments order from largest to smallest</returns>
	std::vector<cFragment> FracturePolygon(cFractureProxy& proxy, const cFractureMaterial& materialConfig, const cFractureImpact& impactConfig)
	{
		cassert(materialConfig.elasticity >= 0.0f);
		cassert(materialConfig.brittleness >= 0.0f);
		cassert(materialConfig.anisotropyFactor >= 0.0f);
		cassert(impactConfig.contactCount >= 0);
		
		/*
		Get fracture settings (calculate fracture range)
		Calculate W(x) (strain energy field) store as weights in the vertices as weights (also get total weight for normalization)
		Create CDF to sample centriods then sample centriods (use a provided fixed seed, maybe based on the fracture proxy ID)
		From the set of centriods, compute the voronoi areas.
		Tessellate the polygon into fragments by clipping the voronoi areas to the polygon
		Return all fragments sorted by size (biggest to smallest)
		*/

		proxy.area = GetPolygonArea(proxy.fragment);
		float density = proxy.mass / proxy.area;
		cassert(density >= 0.0f);
		float rf = computeRf(density, materialConfig);

		int centriodCount = computeCentroidCount(rf, proxy.area, materialConfig);
		// if there is only 1 centriod, we do not shatter, and skip the expensive calculations below
		if (centriodCount < 2)
			return { proxy.fragment };

		computeStrainEnergyField(proxy, materialConfig, impactConfig, rf, density);
		
		// Computes the CDF for selection of centriods
		std::vector<float> cdf(proxy.weights.size(), 0.0f);
		// Compute the cumulative sum
		float cumulative = 0.0f;
		for (size_t i = 0; i < proxy.weights.size(); ++i) {
			cumulative += proxy.weights[i];
			cdf[i] = cumulative;
		}
		// Ensure the last value is exactly 1.0 (precision correction)
		if (!cdf.empty()) {
			cdf.back() = 1.0f;
		}
		
		std::vector<cVec2> centroids = sampleCentroids(proxy, cdf, centriodCount);

		return {};
	}
}
