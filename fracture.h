#pragma once
#include "chioriMath.h"
#include <vector>

namespace chiori
{
	struct cFractureMaterial
	{
		float toughness{ 0.5f };				// Toughness of the material
		float elasticity{ 10.0f };             // Young's Modulus (elasticity)
		float brittleness{ 0.5f };              // Brittleness factor
		vec2 anisotropy{ vec2::zero };			// Anisotropy direction
		float anisotropyFactor{ 0.0f };			// Anisotropy factor
		float k{ 1.0f };						// Scaling factor for fine tuning
		float minPoints{ 1 };					// minimum number of fragments (1 means no fragmentation)
		float maxPoints{ 20 };					// maximum number of fragments
	};
	
	struct cFragment
	{
		std::vector<vec2> vertices;
		vec2 centriod;
		
		cFragment(const std::vector<vec2>& inputVertices) :
			vertices{ inputVertices } {}
	};

	struct cFractureProxy
	{
		cFragment fragment;
		std::vector<float> weights;
		cTransform xf; // transform of the object in world space
		float area;
		float mass;
		int count; // number of vertices

		int id;	// used to generate the seed for the CDF to ensure the same centriods everytime

		cFractureProxy(const std::vector<vec2>& inputVertices, int inID, float inMass = 1.0f) 
			: fragment{ inputVertices }, mass {inMass}, count{static_cast<int>(inputVertices.size())}, id{inID}
		{
			weights.resize(inputVertices.size(), 0.0f);
		}
	};

	struct cFractureImpact
	{
		vec2 localContacts[2];
		vec2 normal;
		float impluseForces[2];
		int contactCount = 0;
	};
	
	float GetPolygonArea(cFragment& inPolygon);

	std::vector<cFragment> FracturePolygon(cFractureProxy& proxy, const cFractureMaterial& materialConfig, const cFractureImpact& impactConfig);
}