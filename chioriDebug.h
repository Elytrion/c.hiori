#pragma once
#include "chioriMath.h"

namespace chiori
{
	struct cDebugColor
	{
		// values will be within 0 - 1
		float r, g, b;
		float a;

		static const cDebugColor Red;
		static const cDebugColor Green;
		static const cDebugColor Blue;
		static const cDebugColor Yellow;
		static const cDebugColor Magenta;
		static const cDebugColor Cyan;
		static const cDebugColor White;
		static const cDebugColor Black;
	};

	inline constexpr cDebugColor cDebugColor::Red = cDebugColor{ 0.9f, 0.1f, 0.1f, 1.0f };
	inline constexpr cDebugColor cDebugColor::Green = cDebugColor{ 0.1f, 0.9f, 0.1f, 1.0f };
	inline constexpr cDebugColor cDebugColor::Blue = cDebugColor{ 0.1f, 0.1f, 0.9f, 1.0f };
	inline constexpr cDebugColor cDebugColor::Yellow = cDebugColor{ 0.9f, 0.9f, 0.1f, 1.0f };
	inline constexpr cDebugColor cDebugColor::Magenta = cDebugColor{ 0.1f, 0.9f, 0.9f, 1.0f };
	inline constexpr cDebugColor cDebugColor::Cyan = cDebugColor{ 0.1f, 0.9f, 0.9f, 1.0f };
	inline constexpr cDebugColor cDebugColor::White = cDebugColor{ 0.9f, 0.9f, 0.9f, 1.0f };
	inline constexpr cDebugColor cDebugColor::Black = cDebugColor{ 0.1f, 0.1f, 0.1f, 1.0f };
	

	class cDebugDraw
	{
	public:
		bool drawShapes{ true };
		bool drawAABBs{ false };
		bool drawTreeAABBs{ false };
		bool drawMass{ true };
		bool drawContactPoints{ false };
		bool drawContactNormals{ false };
		bool drawContactImpulses{ false };
		bool drawFrictionImpulses{ false };
		void* context;
	
		void (*DrawPolygon) (const cVec2* vertices, int vertextCount, cDebugColor color, void* context);
		
		void (*DrawCircle) (cVec2 center, float radius, cDebugColor color, void* context);
	
		void (*DrawPoint) (cVec2 p, float size, cDebugColor color, void* context);

		void (*DrawLine) (cVec2 p1, cVec2 p2, cDebugColor color, void* context);

		void (*DrawString) (cVec2 p, float size, const char* str,  cDebugColor color, void* context);
	
		void (*DrawTransform) (cTransform xf, void* context);
	};
}