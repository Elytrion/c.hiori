#pragma once
#include "voronoi.h"
#include "physicsWorld.h"

namespace chiori
{	
	class cFracturePattern
	{
	public:
		cObjHeader header; // required for pool allocator
		cVec2 min_extent, max_extent;
		cVoronoiDiagram pattern;
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
	public:
		template <typename Allocator = cDefaultAllocator>
		explicit cFractureWorld(Allocator alloc = Allocator()) :
			cPhysicsWorld(alloc),
			f_patterns{ allocator.get() }, f_fractors{ allocator.get() }
		{}

		~cFractureWorld() = default;
		
		cPool<cFracturePattern> f_patterns;
		cPool<cFracturable> f_fractors;
		std::unordered_map<int, cVec2> fractorPointsMap;

		int MakeFracturable(int inActorIndex, cFractureMaterial inMaterial); // turn a regular actor into a fracturable object
		void MakeUnfracturable(int inFractorIndex);
		void SetFracturePattern(int inPatternIndex, int inFractorIndex);
		
		static bool CreateFracturePattern(cFracturePattern& outPattern, const cVoronoiDiagram& inDiagram, const cAABB& inBounds, bool shift = true);
		int CreateNewFracturePattern(const cVoronoiDiagram& inDiagram, const cAABB& inBounds = cAABB());
	
		// call this instead of step when using fractureWorld!
		void f_step(float inFDT, int primaryIterations = 4, int secondaryIterations = 2, bool warmStart = true);
	
	private:
		bool CheckDupePattern(const cVoronoiDiagram& inPattern)
		{
			// check if this pattern is already loaded in
			for (int i = 0; i < f_patterns.size(); i++)
			{
				const cFracturePattern* f_pattern = f_patterns[i];
				const cVoronoiDiagram& ovd = f_pattern->pattern;
				const cVoronoiDiagram& vd = inPattern;

				size_t ovd_v_size = ovd.vertices.size();
				size_t vd_v_size = vd.vertices.size();
				size_t ovd_e_size = ovd.edges.size();
				size_t vd_e_size = vd.edges.size();
				size_t ovd_p_size = ovd.v_points.size();
				size_t vd_p_size = vd.v_points.size();

				if (ovd_v_size == vd_v_size &&
					ovd_e_size == vd_e_size &&
					ovd_p_size == vd_p_size)
				{
					bool sameVerts = (ovd.vertices[0] == vd.vertices[0]) &&
						(ovd.vertices[ovd_v_size - 1] == vd.vertices[ovd_v_size - 1]);

					bool sameEdges = (ovd.edges[0] == vd.edges[0]) &&
						(ovd.edges[ovd_e_size - 1] == vd.edges[ovd_e_size - 1]);

					bool samePoints = (ovd.v_points[0] == vd.v_points[0]) &&
						(ovd.v_points[ovd_p_size - 1] == vd.v_points[ovd_p_size - 1]);

					if (sameVerts && sameEdges && samePoints)
						return true; // duplicate
				}
			}
			return false;
		}
	};
}