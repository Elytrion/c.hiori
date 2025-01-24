#pragma once
#include "chioriDebug.h"
#include "cprocessing.h"

class DebugGraphics
{
private:
	cVec2 cameraCenter{ 0.0f, 0.0f };
	float zoomScale{ 100.0f  };
	cVec2 screenDimensions{ 1600, 900 };
	cDebugDraw draw;
	
public:
	void Create();

	void DrawPolygon(const cVec2* vertices, int vertextCount, cDebugColor color);
	void DrawCircle(cVec2 center, float radius, cDebugColor color);
	void DrawPoint (cVec2 p, float size, cDebugColor color);
	void DrawLine (cVec2 p1, cVec2 p2, cDebugColor color);
	void DrawString (cVec2 p, const char* str, cDebugColor color);
	void DrawTransform (cTransform xf);

	void PanCamera(cVec2 moveDelta);
	void ChangeZoom(float zoom);
	void ChangeScreenDimensions(cVec2 dims);
	void ResetCamera();

	cVec2 ConvertScreenToWorld(cVec2 ps);
	cVec2 ConvertWorldToScreen(cVec2 pw);

	CP_Color ConvertColor(cDebugColor inColor);

};