#pragma once
#include "chioriMath.h"
#include "chioriAllocator.h"
#include <vector>

namespace chiori
{
	struct cFractureProxy
	{
		const vec2* vertices;
		float* weights;
		int count;
		int id;
	};

	struct cFragment
	{
		std::vector<vec2> vertices;
		vec2 centriod;
		float area;
	};

	struct cFractureProxyConfig
	{
		float toughness {0.5f};					// Toughness of the material
		float elasticity{ 100.0f };             // Young's Modulus (elasticity)
		float brittleness{ 0.5f };              // Brittleness factor
		vec2 anisotropy{ vec2::zero };			// Anisotropy direction
		float anisotropyFactor{ 0.0f };			// Anisotropy factor
		float mass{ 1.0f };						// Object mass
		float k{ 1.0f };						// Scaling factor for fine tuning
	};

	class FractureManager
	{
		cAllocator* allocator;	// Custom memory allocator used to allocate and deallocate the memory block.
	public:
		FractureManager(cAllocator* alloc) : allocator{ alloc } {}

		cFractureProxy CreateFractureProxy(vec2* vertices, int count, int id)
		{
			cFractureProxy proxy;
			proxy.vertices = vertices;
			proxy.count = count;
			proxy.id = id;
			proxy.weights = static_cast<float*>(allocator->allocate(count * sizeof(float)));
			// Use placement new to initialize all elements to 0.0f
			for (int i = 0; i < count; ++i) {
				new (proxy.weights + i) float(0.0f); // Placement new with value initialization
			}
			return proxy; // should rvo to be a move
		}
		
		void DestroyFractureProxy(cFractureProxy* proxy)
		{
			// Free the raw memory
			allocator->deallocate(proxy->weights, proxy->count * sizeof(float));
			proxy->weights = nullptr;
			proxy->vertices = nullptr;
		}

		std::vector<cFragment> FracturePolygon(cFractureProxy* proxy, cFractureProxyConfig& config);
	};
}