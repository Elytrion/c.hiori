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
		float mass{ 1.0f };						// Object mass
		float k{ 1.0f };						// Scaling factor for fine tuning
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
		float area;
		float mass;
		int count; // number of vertices

		int id;	// used to generate the seed for the CDF to ensure the same centriods everytime

		cFractureProxy(const std::vector<vec2>& inputVertices, int inID, float inMass = 1.0f) 
			: fragment{ inputVertices }, mass {inMass}, count{inputVertices.size()}, id{inID}
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
		cTransform xf; // transform of the object in world space
	};

	std::vector<cFragment> FracturePolygon(const cFractureProxy& proxy, const cFractureMaterial& materialConfig, const cFractureImpact& impactConfig);
}