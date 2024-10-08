#pragma once

#include "vector2.h"
#include "flag.h"
#include <cstdint>

namespace chiori
{	
	struct Geometry // holds the vertices of a shape, ignoring any and all positional data and scaling data
	{
		enum class GeomType // For future additions of different types of geometry (if any)
		{
			MESH
		};
		GeomType type = GeomType::MESH;
		std::vector<vec2> vertices;
	};

	class Shape // holds a shared pointer to a geometry object and a vector2 for position and scale, and is aware of its local position and scale
	{
	private:
		Flag_8 flags = 0 | (1 << 0) | (1 << 1);
		std::shared_ptr<Geometry> geom;
	public:
		enum
		{
			SIMULATION_SHAPE	= (1 << 0),
			SCENE_QUERY_SHAPE	= (1 << 1),
			TRIGGER_SHAPE		= (1 << 2)
		};	
		
		vec2 localPosition		= vec2::zero;
		vec2 localScale			= vec2::one;
		float localRotation		= 0.0f;

		Shape(std::shared_ptr<Geometry> inGeom, vec2 inLocalPosition = vec2::zero, vec2 inLocalScale = vec2::one, float inLocalRotation = 0.0f)
			: geom(inGeom), localPosition(inLocalPosition), localScale(inLocalScale), localRotation(inLocalRotation) {}
		Shape(std::shared_ptr<Geometry> inGeom) : geom(inGeom) {}

		std::shared_ptr<Geometry> getGeometry() const;
		void setGeometry(std::shared_ptr<Geometry> inGeom);
		
		Flag_8 getFlags() const;
		void setFlags(Flag_8 inFlags);
	};
}