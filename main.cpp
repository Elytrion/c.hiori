
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "cprocessing.h"
#include "graphics.h"
#include "uimanager.h"
#include "scenemanager.h"
#include "physicsWorld.h"
#include "fractureWorld.h"
#include "voronoiSceneManager.h"

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


cFractureWorld world; 
cVoronoiDiagram voronoi;
DebugGraphics p_drawer{ recommendedWidth, recommendedHeight }; // create a graphics instance to draw the world and UI
UIManager ui_manager{ &p_drawer }; // create a ui manager to handle UI input events
SceneManager scene_manager{ &p_drawer, &ui_manager, &world, &voronoi}; // create a scene manager to handle different scenes


void InitChioriGUI()
{
    p_drawer.Create();

    std::vector<cVec2> points;
    int bufferEdgeWidth = 80;
    for (int i = 0; i < 3; i++) {
        float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
        float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth);
        points.push_back({ x, y });
    }
    voronoi.create(points.data(), points.size());
}

void UpdateChioriGUI()
{
    scene_manager.Update(CP_System_GetDt());
}

void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    CP_System_ShowConsole();
    InitChioriGUI();
}

void game_update(void)
{
    CP_Settings_BlendMode(CP_BLEND_ALPHA);
    CP_Graphics_ClearBackground(CP_Color_Create(51, 51, 51, 255));
    CP_Settings_Fill(CP_Color_Create(0, 0, 0, 255));
    
    UpdateChioriGUI();

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

}

int main(void){
    CP_Engine_SetNextGameState(game_init, game_update, game_exit);
    CP_Engine_Run();
    return 0;
}
