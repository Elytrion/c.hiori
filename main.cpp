
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
    //UIComponentConfig testBtnConfig
    //{
    //    50.0f, 50.0f,
    //    80.0f, 30.0f,
    //    0.9f, 0.9f, 0.9f, 1.0f,
    //    "Click Me",
    //    5.0f, 20.0f,
    //    1.0f, 0.0f, 0.0f, 1.0f
    //};
    //// Event for the button
    UIEventTrigger clickEvent;
    clickEvent.type = UIEventType::OnMouseTriggered;
    clickEvent.mouse = MOUSE_BUTTON_1; // Left mouse button
    clickEvent.callback = []() {
        printf("Hello World\n");
        };
    //ui_manager.AddRectUIButton(testBtnConfig, { clickEvent });

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

    UIEventTrigger drawCenterOfMassesTrigger;
    drawCenterOfMassesTrigger.type = UIEventType::OnMouseTriggered;
    drawCenterOfMassesTrigger.mouse = MOUSE_BUTTON_1;
    drawCenterOfMassesTrigger.callback = [&]() {
        printf("WIP, does nothing\n");
		drawer.draw.drawCenterOfMasses = !drawer.draw.drawCenterOfMasses;
        };
    events.push_back(drawCenterOfMassesTrigger);

    std::vector<std::string> buttonNames = {
		"Toggle AABBs",
		"Toggle Tree AABBs",
		"Toggle Mass",
		"Toggle Contact Points",
		"Toggle Contact Normals",
		"Toggle Contact Impulses",
		"Toggle Friction Impulses",
		"Toggle Center of Masses"
	};

    std::vector<cVec2> nameOffsets =
    {
        {-20.0f, 20.0f},
        {10.0f, 20.0f},
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
    for (int i = 0; i < 8; ++i)
    {
        // Define button configuration
        UIComponentConfig buttonConfig{
            startX + spacing * i, y,                    // x, y position
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
