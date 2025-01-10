#include "pch.h"
#include "fracture.h"

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
			const vec2& current = vertices[i];
			const vec2& next = vertices[(i + 1) % n]; // Wrap around to the first vertex
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
		const cFractureImpact& impactConfig, float fractureRange)
	{
		for (int i = 0; i < proxy.count; ++i)
		{

		}
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

		computeStrainEnergyField(proxy, materialConfig, impactConfig, rf);
		
	}
}
