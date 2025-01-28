
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "cprocessing.h"
#include "graphics.h"
#include "uimanager.h"
#include "scenemanager.h"
#include "physicsWorld.h"

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
SceneManager scene_manager{ &drawer, &world }; // create a scene manager to handle different scenes




void RampScene()
{
  
}

void InitUI()
{
    UIComponentConfig testBtnConfig
    {
        50.0f, 50.0f,
        80.0f, 30.0f,
        0.9f, 0.9f, 0.9f, 1.0f,
        "Click Me",
        5.0f, 20.0f,
        1.0f, 0.0f, 0.0f, 1.0f
    };
    // Event for the button
    UIEventTrigger clickEvent;
    clickEvent.type = UIEventType::OnMouseTriggered;
    clickEvent.mouse = MOUSE_BUTTON_1; // Left mouse button
    clickEvent.callback = []() {
        printf("Hello World\n");
        };
    ui_manager.AddRectUIButton(testBtnConfig, { clickEvent });

    float spacing = 50.0f;
    float startX = spacing; // Initial horizontal offset
    float y = 20.0f;        // Fixed Y position near the top of the screen
    cVec2 btnDim = { 30.0f, 30.0f };
    for (int i = 0; i < 8; ++i)
    {
        // Define button configuration
        UIComponentConfig buttonConfig{
            startX, y,                    // x, y position
            btnDim.x, btnDim.y,    // width, height
            0.9f, 0.9f, 0.9f, 1.0f,           // RGBA color
            "Button " + std::to_string(i + 1), // Button text
            18.0f                         // Text size
        };
    }
}

void InitChioriGUI()
{
    drawer.Create();
    InitUI();
    //CP_System_SetWindowSize(800, 450);
    //drawer.ChangeScreenDimensions({ 800, 450 });
}

void UpdateChioriGUI()
{
    ui_manager.Update();
    scene_manager.Update(CP_System_GetDt());

    if (CP_Input_KeyTriggered(KEY_EQUAL))
    {
        int index = (scene_manager.currentScene + 1) % scene_manager.sceneCount;
        scene_manager.ChangeScene(index);
    }
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
