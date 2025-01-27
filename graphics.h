#pragma once
#include "chioriDebug.h"
#include "cprocessing.h"

class DebugGraphics
{
private:
	chiori::cVec2 cameraCenter{ 0.0f, 0.0f };
	float zoomScale{ 50.0f  };
	chiori::cVec2 screenDimensions{ 1600, 900 };

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
	void ChangeZoom(float zoom);
	void ChangeScreenDimensions(chiori::cVec2 dims);
	void ResetCamera();

	chiori::cVec2 ConvertScreenToWorld(chiori::cVec2 ps);
	chiori::cVec2 ConvertWorldToScreen(chiori::cVec2 pw);

	CP_Color ConvertColor(chiori::cDebugColor inColor);

};