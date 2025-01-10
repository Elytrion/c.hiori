#include "pch.h"
#include "fracture.h"

namespace chiori
{

	/// <summary>
	/// Fractures a given polygon detailed in cFractureProxy into smaller polygons based on the material and impact configs
	/// </summary>
	/// <param name="proxy">- Polygon to be fractured</param>
	/// <param name="materialConfig">- Material properties of the polygon</param>
	/// <param name="impactConfig">- Collision impulses applied to the polygon</param>
	/// <returns>A vector of fragments order from largest to smallest</returns>
	std::vector<cFragment> FracturePolygon(const cFractureProxy& proxy, const cFractureMaterial& materialConfig, const cFractureImpact& impactConfig)
	{
		/*
		Get fracture settings (calculate fracture range)
		Calculate W(x) (strain energy field) store as weights in the vertices as weights (also get total weight for normalization)
		Create CDF to sample centriods then sample centriods (use a provided fixed seed, maybe based on the fracture proxy ID)
		From the set of centriods, compute the voronoi areas.
		Tessellate the polygon into fragments by clipping the voronoi areas to the polygon
		Return all fragments sorted by size (biggest to smallest)
		*/
		
	}
}
