#pragma once
#include "chioriDebug.h"
#include "cprocessing.h"


struct UIElement
{
	enum ElementType
	{
		RECT,
		CIRCLE,
		TEXT,
		LINE
	};

	char textBuffer[64];     // Text for TEXT elements
	CP_Color color;		     // Color of the element
	float position[2];       // Position in screen space (Start position for LINE)
	float size[2];           // Size for RECT, radius for CIRCLE (stored in `size.x`), end position for LINE
	ElementType type;
};

class DebugGraphics
{
private:
	chiori::cVec2 cameraCenter{ 0.0f, 0.0f };
	float zoomScale{ 50.0f  };
	chiori::cVec2 screenDimensions{ 1600, 900 };
	chiori::cVec2 initScreenDimensions{ 1600, 900 };
	std::vector<UIElement> uiElements; // Cache of UI elements


	
public:
	DebugGraphics(float screenWidth, float screenHeight) : screenDimensions(screenWidth, screenHeight), initScreenDimensions(screenWidth, screenHeight) {}
	float getZoom() { return zoomScale; }
	chiori::cVec2 getCameraWorldPos();
	chiori::cVec2 getCameraScreenPos();
	chiori::cVec2 getScreenDimensions() { return screenDimensions;}
	chiori::cDebugDraw draw;
	void Create();

	void DrawPolygon(const chiori::cVec2* vertices, int vertextCount, chiori::cDebugColor color);
	void DrawCircle(chiori::cVec2 center, float radius, chiori::cDebugColor color);
	void DrawPoint (chiori::cVec2 p, float size, chiori::cDebugColor color);
	void DrawLine (chiori::cVec2 p1, chiori::cVec2 p2, chiori::cDebugColor color);
	void DrawString (chiori::cVec2 p, float size, const char* str, chiori::cDebugColor color);
	void DrawTransform (chiori::cTransform xf);

	void PanCamera(chiori::cVec2 moveDelta);
	void SetCamera(chiori::cVec2 screenPos);
	void ChangeZoom(float zoom);
	void ChangeScreenDimensions(chiori::cVec2 dims);
	void ResetCamera();

	chiori::cVec2 ConvertScreenToWorld(chiori::cVec2 ps);
	chiori::cVec2 ConvertWorldToScreen(chiori::cVec2 pw);

	CP_Color ConvertColor(chiori::cDebugColor inColor);

	int AddUIRect(chiori::cVec2 position, chiori::cVec2 size, chiori::cDebugColor color);
	void DrawUIRect(float x, float y, float w, float h, CP_Color color);
	int AddUICircle(chiori::cVec2 center, float radius, chiori::cDebugColor color);
	void DrawUICircle(float x, float y, float radius, CP_Color color);
	int AddUIText(chiori::cVec2 position, const std::string& str, float size, chiori::cDebugColor color);
	void DrawUIText(float x, float y, const std::string& str, float size, CP_Color color);
	int AddUILine(chiori::cVec2 start, chiori::cVec2 end, chiori::cDebugColor color);
	void DrawUILine(float x1, float y1, float x2, float y2, CP_Color color);

	void DrawPhysicsWorld(void* world);
	void DrawUI(); // always call this AFTER draw frame

	void ClearUI() { uiElements.clear(); }

	std::vector<UIElement>& getUIElements() { return uiElements; }
};