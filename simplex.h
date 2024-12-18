#pragma once

#include "chioriMath.h"
#include <array>

namespace chiori
{
	struct Mvert // Minkowski Vertex, a vertex on the Configuration Space Object (CSO)
	{
		vec2 a{ 0,0 }; // support point on obj A
		vec2 b{ 0,0 }; // support point on obj B
		vec2 w{ 0,0 }; // support point on CSO aka minkowski difference between points a and b (a - b)
		float l{ -1.0f }; // barycentric
		Mvert(const vec2& inA = vec2::zero, const vec2& inB = vec2::zero) : a(inA), b(inB), w(inA - inB) {}
		
		friend std::ostream& operator<<(std::ostream& inOS, const Mvert& inVec) {
			inOS << "mvert( \n";
			inOS << " a: " << inVec.a << " | ";
			inOS << " b: " << inVec.b << " | ";
			inOS << " w: " << inVec.w << " | ";
			inOS << ")";
			return inOS;
		}
	};

	template <unsigned N>
	class NSimplex
	{
	private:
		std::array<Mvert, N> s_points;

	public:
		unsigned s_size;
		NSimplex() : s_points{}, s_size{0} {};

		NSimplex& operator=(std::initializer_list<Mvert> list)
		{
			for (auto point = list.begin(); point != list.end(); point++)
			{
				s_points[std::distance(list.begin(), point)] = *point;
			}
			s_size = list.size();
			return *this;
		}

		NSimplex& operator=(const NSimplex& inSimplex)
		{
			for (unsigned i = 0; i < inSimplex.size(); i++)
			{
				s_points[i] = inSimplex[i];
			}
			s_size = inSimplex.size();
			return *this;
		}

		void resize(unsigned size)
		{
			s_size = min(size, N);
		}

		void push_front(Mvert point)
		{
			s_points = { point, s_points[0], s_points[1] };
			s_size = min(s_size + 1, N);
		}

		void clear() { s_size = 0; }

		bool isDupe(const Mvert& v)
		{
			if (s_size <= 0)
				return false;
			
			for (unsigned i = 0; i < s_size; i++){
				if (v.a == s_points[i].a && v.b == s_points[i].b && v.w == s_points[i].w)
					return true;
			}
			return false;
		}

		Mvert& operator[](unsigned i) { return s_points[i]; }
		const Mvert& operator[](unsigned i) const { return s_points[i]; }
		unsigned size() const { return s_size; }
		auto begin() const { return s_points.begin(); }
		auto end() const { return s_points.end() - (N - s_size); }
	};

	using Simplex = NSimplex<3>; // we are working in 2D, so the simplex max size is dimension + 1 -> 2 + 1 -> 3

	struct Edge
	{
		Edge() : distance{ 0.0f }, normal{ vec2::zero }, index{ 0 } {}
		Edge(float d, const vec2& n, int i) : distance{ d }, normal{ n }, index{ i } {}

		float distance;     // The distance of the edge from the origin
		vec2 normal;		// The normal of the edge
		int index;          // The index of the edge (the index of the second vertex that makes up the edge in the polytope)
	};
}