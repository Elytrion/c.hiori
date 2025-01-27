#pragma once
#include "chioriDebug.h"
#include "cprocessing.h"


struct UIElement
{
	enum ElementType
	{
		RECT,
		CIRCLE,
		TEXT
	};

	char textBuffer[64];     // Text for TEXT elements
	CP_Color color;		     // Color of the element
	float position[2];       // Position in screen space
	float size[2];           // Size for RECT, radius for CIRCLE (stored in `size.x`)
	ElementType type;
};

class DebugGraphics
{
private:
	chiori::cVec2 cameraCenter{ 0.0f, 0.0f };
	float zoomScale{ 50.0f  };
	chiori::cVec2 screenDimensions{ 1600, 900 };
	std::vector<UIElement> uiElements; // Cache of UI elements

	void DrawUI();
	
public:
	
	chiori::cDebugDraw draw;
	void Create();

	void DrawPolygon(const chiori::cVec2* vertices, int vertextCount, chiori::cDebugColor color);
	void DrawCircle(chiori::cVec2 center, float radius, chiori::cDebugColor color);
	void DrawPoint (chiori::cVec2 p, float size, chiori::cDebugColor color);
	void DrawLine (chiori::cVec2 p1, chiori::cVec2 p2, chiori::cDebugColor color);
	void DrawString (chiori::cVec2 p, const char* str, chiori::cDebugColor color);
	void DrawTransform (chiori::cTransform xf);

	void PanCamera(chiori::cVec2 moveDelta);
	void SetCamera(chiori::cVec2 screenPos);
	void ChangeZoom(float zoom);
	void ChangeScreenDimensions(chiori::cVec2 dims);
	void ResetCamera();

	chiori::cVec2 ConvertScreenToWorld(chiori::cVec2 ps);
	chiori::cVec2 ConvertWorldToScreen(chiori::cVec2 pw);

	CP_Color ConvertColor(chiori::cDebugColor inColor);

	void DrawUIRect(chiori::cVec2 position, chiori::cVec2 size, chiori::cDebugColor color);
	void DrawUICircle(chiori::cVec2 center, float radius, chiori::cDebugColor color);
	void DrawUIText(chiori::cVec2 position, const std::string& str, chiori::cDebugColor color);

	void DrawFrame(void* world);
};