#include "pch.h"
#include "graphics.h"
#include "physicsWorld.h"



using namespace chiori;


void DrawPolygonFnc(const cVec2* vertices, int vertexCount, cDebugColor color, void* context)
{
	static_cast<DebugGraphics*>(context)->DrawPolygon(vertices, vertexCount, color);
}

void DrawCircleFnc(cVec2 center, float radius, cDebugColor color, void* context)
{
	static_cast<DebugGraphics*>(context)->DrawCircle(center, radius, color);
}

void DrawPointFnc(cVec2 p, float size, cDebugColor color, void* context)
{
	static_cast<DebugGraphics*>(context)->DrawPoint(p, size, color);
}

void DrawLineFnc (cVec2 p1, cVec2 p2, cDebugColor color, void* context)
{
	static_cast<DebugGraphics*>(context)->DrawLine(p1, p2, color);
}

void DrawStringFnc(cVec2 p, const char* str, cDebugColor color, void* context)
{
	static_cast<DebugGraphics*>(context)->DrawString(p, str, color);
}

void DrawTransformFnc(cTransform xf, void* context)
{
	static_cast<DebugGraphics*>(context)->DrawTransform(xf);
}

void DebugGraphics::Create()
{
	draw.DrawPolygon = DrawPolygonFnc;
	draw.DrawCircle = DrawCircleFnc;
	draw.DrawPoint = DrawPointFnc;
	draw.DrawLine = DrawLineFnc;
	draw.DrawString = DrawStringFnc;
	draw.DrawTransform = DrawTransformFnc;
	draw.context = this;
}

void DebugGraphics::ResetCamera()
{
	cameraCenter = cVec2{ 0.0f, 0.0f };  // Center the camera at the origin
	zoomScale =100.0f; // Default zoom scale
}

void DebugGraphics::PanCamera(cVec2 moveDelta)
{
	cameraCenter += moveDelta / zoomScale; // Adjust for current zoom
}

void DebugGraphics::SetCamera(chiori::cVec2 screenPos)
{
	cameraCenter = screenPos;
}

void DebugGraphics::ChangeZoom(float zoom)
{
	zoomScale = c_clamp(zoom, 1.0f, 1000.0f); // Clamp to avoid excessive zoom
}

void DebugGraphics::ChangeScreenDimensions(cVec2 dims)
{
	screenDimensions = dims;
}

cVec2 DebugGraphics::ConvertScreenToWorld(cVec2 ps)
{
	cVec2 halfScreen = screenDimensions * 0.5f;
	// Flip Y-axis by subtracting from screen height
	ps.y = screenDimensions.y - ps.y;
	return (ps - halfScreen) / zoomScale + cameraCenter;
}
cVec2 DebugGraphics::ConvertWorldToScreen(cVec2 pw)
{
	cVec2 halfScreen = screenDimensions * 0.5f;
	// Convert world to screen and flip Y-axis
	cVec2 screenPos = (pw - cameraCenter) * zoomScale + halfScreen;
	screenPos.y = screenDimensions.y - screenPos.y;
	return screenPos;
}

CP_Color DebugGraphics::ConvertColor(cDebugColor inColor)
{
	int r = static_cast<int>(inColor.r * 255.0f);
	int g = static_cast<int>(inColor.g * 255.0f);
	int b = static_cast<int>(inColor.b * 255.0f);
	int a = static_cast<int>(inColor.a * 255.0f);
	return CP_Color_Create(r, g, b, a);
}

void DebugGraphics::DrawPolygon(const cVec2* vertices, int vertexCount, cDebugColor color)
{
	for (int i = 0; i < vertexCount; ++i)
	{
		int next = (i + 1) % vertexCount;
		cVec2 start = ConvertWorldToScreen(vertices[i]);
		cVec2 end = ConvertWorldToScreen(vertices[next]);
		
		CP_Color cpColor = ConvertColor(color);
		CP_Settings_Fill(cpColor);
		CP_Graphics_DrawCircle(start.x, start.y, 2);
		CP_Settings_StrokeWeight(2);
		CP_Settings_Stroke(cpColor);
		CP_Graphics_DrawLine(start.x, start.y, end.x, end.y);
	}
}
void DebugGraphics::DrawCircle(cVec2 center, float radius, cDebugColor color)
{
	// Convert center to screen coordinates
	cVec2 screenCenter = ConvertWorldToScreen(center);

	// Scale the radius based on zoom
	float screenRadius = radius * zoomScale;

	// Set the color
	CP_Color cpColor = ConvertColor(color);
	CP_Settings_Fill(cpColor);

	// Draw the circle
	CP_Graphics_DrawCircle(screenCenter.x, screenCenter.y, screenRadius);
}

void DebugGraphics::DrawPoint(cVec2 p, float size, cDebugColor color)
{
	// Convert point to screen coordinates
	cVec2 screenPoint = ConvertWorldToScreen(p);

	// Set the color
	CP_Color cpColor = ConvertColor(color);
	CP_Settings_Fill(cpColor);

	// Draw the point as a small circle
	CP_Graphics_DrawCircle(screenPoint.x, screenPoint.y, size);
}
void DebugGraphics::DrawLine(cVec2 p1, cVec2 p2, cDebugColor color)
{
	// Convert points to screen coordinates
	cVec2 screenP1 = ConvertWorldToScreen(p1);
	cVec2 screenP2 = ConvertWorldToScreen(p2);

	// Set the color and stroke
	CP_Color cpColor = ConvertColor(color);
	CP_Settings_Stroke(cpColor);
	CP_Settings_StrokeWeight(3);

	// Draw the line
	CP_Graphics_DrawLine(screenP1.x, screenP1.y, screenP2.x, screenP2.y);
}
void DebugGraphics::DrawString(cVec2 p, const char* str, cDebugColor color)
{
	// Convert position to screen coordinates
	cVec2 screenPos = ConvertWorldToScreen(p);

	// Set the color
	CP_Color cpColor = ConvertColor(color);
	CP_Settings_Fill(cpColor);

	// Draw the string
	CP_Font_DrawText(str, screenPos.x, screenPos.y);
}
void DebugGraphics::DrawTransform(cTransform xf)
{
	// Convert origin to screen coordinates
	cVec2 screenOrigin = ConvertWorldToScreen(xf.p);

	// fixed axis length in screen space 
	float fixedLength = 25.0f;
	float inverseZoom = 1.0f / zoomScale;

	// Scale the axis vectors inversely to zoom
	cVec2 xAxis = xf.p + cVec2{ fixedLength * inverseZoom, 0.0f }.rotated(xf.q);
	cVec2 yAxis = xf.p + cVec2{ 0.0f, fixedLength * inverseZoom }.rotated(xf.q);

	// Convert axes to screen coordinates
	cVec2 screenXAxis = ConvertWorldToScreen(xAxis);
	cVec2 screenYAxis = ConvertWorldToScreen(yAxis);

	// Draw the axes
	// X-axis in red
	CP_Settings_Stroke(CP_Color_Create(255, 0, 0, 255));
	CP_Settings_StrokeWeight(2);
	CP_Graphics_DrawLine(screenOrigin.x, screenOrigin.y, screenXAxis.x, screenXAxis.y);

	// Y-axis in green
	CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255));
	CP_Settings_StrokeWeight(2);
	CP_Graphics_DrawLine(screenOrigin.x, screenOrigin.y, screenYAxis.x, screenYAxis.y);

	// Draw the origin as a small circle
	CP_Settings_Fill(CP_Color_Create(255, 255, 255, 255));
	CP_Graphics_DrawCircle(screenOrigin.x, screenOrigin.y, 4);
}


void DebugGraphics::DrawUIRect(cVec2 position, cVec2 size, cDebugColor color)
{
	UIElement newElement;
	newElement.type = UIElement::ElementType::RECT;
	newElement.position[0] = position.x;
	newElement.position[1] = position.y;
	newElement.size[0] = size.x;
	newElement.size[1] = size.y;
	newElement.color = ConvertColor(color);
	uiElements.push_back(newElement);
}

void DebugGraphics::DrawUICircle(cVec2 center, float radius, cDebugColor color)
{
	UIElement newElement;
	newElement.type = UIElement::ElementType::CIRCLE;
	newElement.position[0] = center.x;
	newElement.position[1] = center.y;
	newElement.size[0] = radius;
	newElement.color = ConvertColor(color);
	uiElements.push_back(newElement);
}

void DebugGraphics::DrawUIText(cVec2 position, const std::string& str, cDebugColor color)
{
	UIElement newElement;
	newElement.type = UIElement::ElementType::TEXT;
	newElement.position[0] = position.x;
	newElement.position[1] = position.y;
	strncpy_s(newElement.textBuffer, str.c_str(), 63);
	newElement.textBuffer[63] = '\0';
	uiElements.push_back(newElement);
}

void DebugGraphics::DrawUI()
{
	for (int i = 0; i < uiElements.size(); ++i)
	{
		const UIElement& uie = uiElements[i];
		CP_Settings_Fill(uie.color);
		switch (uie.type)
		{
		case UIElement::ElementType::RECT:
			CP_Graphics_DrawRect(uie.position[0], uie.position[1], uie.size[0], uie.size[1]);
			break;
		case UIElement::ElementType::CIRCLE:
			CP_Graphics_DrawCircle(uie.position[0], uie.position[1], uie.size[0]);
			break;
		case UIElement::ElementType::TEXT:
			CP_Font_DrawText(uie.textBuffer, uie.position[0], uie.position[1]);
			break;
		}
	}
}

void DebugGraphics::DrawFrame(void* world)
{
	if (world)
	{
		cPhysicsWorld* pworld = static_cast<cPhysicsWorld*>(world);

		pworld->DebugDraw(&draw);
	}

	DrawUI();
}



