#pragma once
#include "voronoi.h"
#include "physicsWorld.h"

namespace chiori
{	
	class cFracturePattern
	{
	public:
		cObjHeader header; // required for pool allocator
	
		cFracturePattern() { };
		cFracturePattern(const std::string& filePath);

		std::string filePath;
		cVoronoiDiagram pattern;

		void load(const std::string& inFilePath = "");
		void save(const std::string& inFilePath = "");
	};
	
	struct cFractureMaterial
	{
		float toughness{ 0.5f };				// Toughness of the material.
		float elasticity{ 10.0f };              // Young's Modulus (elasticity). 
		float brittleness{ 0.5f };              // Brittleness factor. 
		cVec2 anisotropy{ cVec2::zero };		// Anisotropy direction
		float anisotropyFactor{ 0.0f };			// Anisotropy factor
		float k{ 1.0f };						// Scaling factor for custom fine tuning
	};

	class cFracturable
	{
	public:
		cObjHeader header; // required for pool allocator
		
		cFractureMaterial f_material; // material to use to fracture object

		int actorIndex{ -1 };
		int patternIndex{ -1 };
		bool onceFracturable{ true }; // if true, child fragments will not be cFracturables, instead normal actors. 

		float minRRot{ 0.0f }; // the maximum rotation used to randomize the fracture pattern
		float maxRRot{ 0.0f }; // the minimum rotation used to randomize the fracture pattern
	};


	class cFractureWorld : public cPhysicsWorld
	{
		cPool<cFracturePattern> f_patterns;
		cPool<cFracturable> f_fractors;

		void f_step(float inFDT); // call this DIRECTLY after step to allow for fracture checks!
		int MakeFracturable(int inActorIndex); // turn a regular actor into a fracturable object

		void SetFracturePattern(int inPatternIndex, int inFractorIndex);
		int LoadFracturePattern(const std::string& inFilePath);
		void SaveFracturePattern(int inPatternIndex, const std::string& inFilePath);
	};
}