
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "cprocessing.h"
#include "graphics.h"
#include "uimanager.h"
#include "scenemanager.h"
#include "physicsWorld.h"
#include "voronoi.h"

using namespace chiori;

CP_Color randomColors[] = {
    { 128, 0,   0,   255 },
    { 128, 128, 0,   255 },
    { 0,   128, 0,   255 },
    { 0,   128, 128, 255 },
    { 0,   0,   128, 255 },
    { 128, 0,   128, 255 } };
float recommendedWidth = 1600.0f;
float recommendedHeight = 900.0f;
bool drawFPS = true;
cVec2 middle = cVec2{ recommendedWidth / 2.0f, recommendedHeight - 100.0f };

cPhysicsWorld world; // create an instance of the physics world
DebugGraphics drawer{ recommendedWidth, recommendedHeight }; // create a graphics instance to draw the world and UI
UIManager ui_manager{ &drawer }; // create a ui manager to handle UI input events
SceneManager scene_manager{ &drawer, &ui_manager, &world }; // create a scene manager to handle different scenes

#pragma region c.hiori GUI
void InitUI()
{
    std::vector<UIEventTrigger> events;
    UIEventTrigger drawAABBtrigger;
    drawAABBtrigger.type = UIEventType::OnMouseTriggered;
    drawAABBtrigger.mouse = MOUSE_BUTTON_1;
    drawAABBtrigger.callback = [&]() {
        printf("Toggling AABB display\n");
        drawer.draw.drawAABBs = !drawer.draw.drawAABBs;
        };
    events.push_back(drawAABBtrigger);

    UIEventTrigger drawTreeAABBtrigger;
    drawTreeAABBtrigger.type = UIEventType::OnMouseTriggered;
    drawTreeAABBtrigger.mouse = MOUSE_BUTTON_1;
    drawTreeAABBtrigger.callback = [&]() {
        printf("Toggling Tree AABB display\n");
		drawer.draw.drawTreeAABBs = !drawer.draw.drawTreeAABBs;
		};
    events.push_back(drawTreeAABBtrigger);

    UIEventTrigger drawMassTrigger;
    drawMassTrigger.type = UIEventType::OnMouseTriggered;
    drawMassTrigger.mouse = MOUSE_BUTTON_1;
    drawMassTrigger.callback = [&]() {
        printf("Toggling transform/mass display\n");
        drawer.draw.drawMass = !drawer.draw.drawMass;
        };
    events.push_back(drawMassTrigger);

    UIEventTrigger drawContactPointsTrigger;
    drawContactPointsTrigger.type = UIEventType::OnMouseTriggered;
    drawContactPointsTrigger.mouse = MOUSE_BUTTON_1;
    drawContactPointsTrigger.callback = [&]() {
        printf("Toggling contact point display\n");
		drawer.draw.drawContactPoints = !drawer.draw.drawContactPoints;
		};
    events.push_back(drawContactPointsTrigger);

    UIEventTrigger drawContactNormalsTrigger;
    drawContactNormalsTrigger.type = UIEventType::OnMouseTriggered;
    drawContactNormalsTrigger.mouse = MOUSE_BUTTON_1;
    drawContactNormalsTrigger.callback = [&]() {
        printf("Toggling contact normal display\n");
        drawer.draw.drawContactNormals = !drawer.draw.drawContactNormals;
		};
    events.push_back(drawContactNormalsTrigger);

    UIEventTrigger drawContactImpulsesTrigger;
    drawContactImpulsesTrigger.type = UIEventType::OnMouseTriggered;
    drawContactImpulsesTrigger.mouse = MOUSE_BUTTON_1;
    drawContactImpulsesTrigger.callback = [&]() {
        printf("Toggling contact impulse display\n");
		drawer.draw.drawContactImpulses = !drawer.draw.drawContactImpulses;
        };
    events.push_back(drawContactImpulsesTrigger);

    UIEventTrigger drawFrictionImpulsesTrigger;
    drawFrictionImpulsesTrigger.type = UIEventType::OnMouseTriggered;
    drawFrictionImpulsesTrigger.mouse = MOUSE_BUTTON_1;
    drawFrictionImpulsesTrigger.callback = [&]() {
        printf("Toggling friction impulse display\n");
        drawer.draw.drawFrictionImpulses = !drawer.draw.drawFrictionImpulses;
		};
    events.push_back(drawFrictionImpulsesTrigger);

    std::vector<std::string> buttonNames = {
		"Toggle AABBs",
		"Toggle Tree AABBs",
		"Toggle Mass",
		"Toggle Contact Points",
		"Toggle Contact Normals",
		"Toggle Contact Impulses",
		"Toggle Friction Impulses"
	};

    std::vector<cVec2> nameOffsets =
    {
        {-20.0f, 20.0f},
        {10.0f, 20.0f},
        {10.0f, 20.0f},
        {10.0f, 20.0f},
        {10.0f, 20.0f},
        {10.0f, 20.0f},
        {10.0f, 20.0f}
    };

    float spacing = 100.0f;
    float startX = spacing; // Initial horizontal offset
    float y = 40.0f;        // Fixed Y position near the top of the screen
    cVec2 btnDim = { 30.0f, 30.0f };
    std::vector<int> indices = {};
    for (int i = 0; i < 7; ++i)
    {
        // Define button configuration
        UIComponentConfig buttonConfig{
            startX + spacing * i, y,    // x, y position
            btnDim.x, btnDim.y,         // width, height
            0.9f, 0.9f, 0.9f, 1.0f,     // RGBA color
            buttonNames[i],      // Button text
            nameOffsets[i].x, nameOffsets[i].y,                // Text offset
            0.1f, 0.1f, 0.1f, 1.0f,     // Text color
            15.0f                       // Text size
        };
        ui_manager.AddRectUIButton(buttonConfig, { events[i]});
    }




}

void InitChioriGUI()
{
    drawer.Create();
    InitUI();
}

void UpdateChioriGUI()
{
    scene_manager.Update(CP_System_GetDt());

    if (CP_Input_KeyTriggered(KEY_EQUAL))
    {
        int index = (scene_manager.currentScene + 1) % scene_manager.sceneCount;
        scene_manager.ChangeScene(index);
    }
}
#pragma endregion

std::vector<cVec2> used_points;
std::vector<cVec2> points;
std::vector<std::vector<cVec2>> tris;
int numPoints = 70;
int initialNumPoints = 10;
cVoronoiDiagram voronoi;

void SetupVoronoi()
{
    // Generate random points for testing
    int bufferEdgeWidth = 100;
    for (int i = 0; i < numPoints; i++) {
        float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
        float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth);
        if (i < initialNumPoints)
            used_points.push_back({ x, y });
        else
            points.push_back({ x, y });
    }
    voronoi.create(used_points.data(), used_points.size());
    tris = cVoronoiDiagram::triangulateDelaunator(used_points);

    //cVoronoiDiagram vd;
    //vd.create(points.data(), points.size());

    //cVoronoiDiagram::SaveVoronoiDiagram("kringles", vd);
    //cVoronoiDiagram cvd = cVoronoiDiagram::LoadVoronoiDiagram("kringles");
}

bool drawTriangles = false;
bool drawVVerts = true;
bool drawCentriod = false;
bool drawPoints = true;
void DrawVoronoi()
{
    if (drawTriangles)
    { // delaunay drawing
        CP_Settings_StrokeWeight(1);
        CP_Settings_Stroke(CP_Color_Create(255, 0, 0, 255)); // Red for Delaunay edges

        for (const auto& tri : tris) {
            cVec2 a = tri[0];
            cVec2 b = tri[1];
            cVec2 c = tri[2];
            CP_Graphics_DrawLine(a.x, a.y, b.x, b.y);
            CP_Graphics_DrawLine(b.x, b.y, c.x, c.y);
            CP_Graphics_DrawLine(c.x, c.y, a.x, a.y);
        }
    }

    if (drawVVerts)
    { // voronoi drawing

        CP_Color cellColor = CP_Color_Create(255, 255, 255, 255);
        for (const auto& vvertex : voronoi.vertices)
        {
            for (const auto& edgeIndex : vvertex.edgeIndices)
            {
                const cVEdge& edge = voronoi.edges[edgeIndex];
                if (edge.infinite)
                {
                    const cVec2& p1 = edge.origin;
                    const cVec2& p2 = edge.origin + edge.endDir * 100;
                    CP_Settings_StrokeWeight(1);
                    CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255)); // Green for Delaunay edges
                    CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                    continue;
                }
                const cVec2& p1 = edge.origin;
                const cVec2& p2 = edge.endDir;
                CP_Settings_StrokeWeight(1);
                CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255)); // Green for Delaunay edges
                CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
            }

            if (drawCentriod)
            {
                CP_Settings_Fill(CP_Color_Create(0, 255, 100, 255));
                CP_Graphics_DrawCircle(vvertex.site.x, vvertex.site.y, 10);
            }
        }
    }

    if (!drawPoints)
        return;

    for (const auto& p : used_points) {
        CP_Settings_Fill(CP_Color_Create(255, 255, 0, 255));
        CP_Graphics_DrawCircle(p.x, p.y, 5);
    }

	for (const auto& p : points) {
        CP_Settings_Fill(CP_Color_Create(0, 255, 255, 255));
        CP_Graphics_DrawCircle(p.x, p.y, 5);
    }
}

float timer = 0.5f;
void UpdateVoronoi()
{

    if (CP_Input_KeyTriggered(KEY_EQUAL))
    {
       used_points.clear();
       points.clear();
       SetupVoronoi();
    }


    if (CP_Input_KeyTriggered(KEY_D))
    {
        drawTriangles = !drawTriangles;
    }
    if (CP_Input_KeyTriggered(KEY_V))
    {
        drawVVerts = !drawVVerts;
    }
    if (CP_Input_KeyTriggered(KEY_P))
    {
        drawPoints = !drawPoints;
    }
    if (CP_Input_KeyTriggered(KEY_C))
    {
        drawCentriod = !drawCentriod;
    }
    DrawVoronoi();

    if (used_points.size() < numPoints)
    {
        timer -= CP_System_GetDt();
        if (timer <= 0.0f)
        {
            timer = 0.05f;
            // move the last point of points to used_points
            // then retriangulate
            used_points.push_back(std::move(points.front()));
            points.erase(points.begin());
            tris.clear();
            voronoi.vertices.clear();
            tris = cVoronoiDiagram::triangulateDelaunator(used_points);
            voronoi.create(used_points.data(), used_points.size());
        }
    }

}






void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    CP_System_ShowConsole();
    InitChioriGUI();
    SetupVoronoi();
}

bool drawChiori = true;
void game_update(void)
{
    CP_Settings_BlendMode(CP_BLEND_ALPHA);
    CP_Graphics_ClearBackground(CP_Color_Create(51, 51, 51, 255));
    CP_Settings_Fill(CP_Color_Create(0, 0, 0, 255));
    
    if (CP_Input_KeyTriggered(KEY_9))
    {
        drawChiori = !drawChiori;
	}

    if (drawChiori)
        UpdateChioriGUI();
    else
        UpdateVoronoi();
    


    // Profiling info and frameRate testing
    if (true)
    {
        CP_Settings_TextSize(20);
        CP_Settings_BlendMode(CP_BLEND_ALPHA);
        CP_Settings_Fill(CP_Color_Create(0, 0, 0, 128));
        CP_Settings_NoStroke();
        CP_Graphics_DrawRect(0, 0, 150, 30);
        CP_Settings_Fill(CP_Color_Create(255, 255, 255, 255));
        char buffer[100];
        sprintf_s(buffer, 100, "FPS: %f", CP_System_GetFrameRate());
        CP_Font_DrawText(buffer, 20, 20);
    }

}



void game_exit(void)
{
 //   if (voronoi != nullptr) {
	//	delete voronoi;
	//}
}

int main(void){
    CP_Engine_SetNextGameState(game_init, game_update, game_exit);
    CP_Engine_Run();
    return 0;
}
