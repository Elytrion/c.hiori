
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

cFractureWorld world; // create an instance of the physics world
DebugGraphics p_drawer{ recommendedWidth, recommendedHeight }; // create a graphics instance to draw the world and UI
UIManager ui_manager{ &p_drawer }; // create a ui manager to handle UI input events
SceneManager scene_manager{ &p_drawer, &ui_manager, &world }; // create a scene manager to handle different scenes

DebugGraphics v_drawer{ recommendedWidth, recommendedHeight }; // create graphics instance to draw voronoi 
VoronoiSceneManager v_scene_manager{ &v_drawer }; // create a specialized scene manager for voronoi scenes

#pragma region c.hiori GUI
void InitUI()
{
    std::vector<UIEventTrigger> events;
    UIEventTrigger drawAABBtrigger;
    drawAABBtrigger.type = UIEventType::OnMouseTriggered;
    drawAABBtrigger.mouse = MOUSE_BUTTON_1;
    drawAABBtrigger.callback = [&]() {
        printf("Toggling AABB display\n");
        p_drawer.draw.drawAABBs = !p_drawer.draw.drawAABBs;
        };
    events.push_back(drawAABBtrigger);

    UIEventTrigger drawTreeAABBtrigger;
    drawTreeAABBtrigger.type = UIEventType::OnMouseTriggered;
    drawTreeAABBtrigger.mouse = MOUSE_BUTTON_1;
    drawTreeAABBtrigger.callback = [&]() {
        printf("Toggling Tree AABB display\n");
		p_drawer.draw.drawTreeAABBs = !p_drawer.draw.drawTreeAABBs;
		};
    events.push_back(drawTreeAABBtrigger);

    UIEventTrigger drawMassTrigger;
    drawMassTrigger.type = UIEventType::OnMouseTriggered;
    drawMassTrigger.mouse = MOUSE_BUTTON_1;
    drawMassTrigger.callback = [&]() {
        printf("Toggling transform/mass display\n");
        p_drawer.draw.drawMass = !p_drawer.draw.drawMass;
        };
    events.push_back(drawMassTrigger);

    UIEventTrigger drawContactPointsTrigger;
    drawContactPointsTrigger.type = UIEventType::OnMouseTriggered;
    drawContactPointsTrigger.mouse = MOUSE_BUTTON_1;
    drawContactPointsTrigger.callback = [&]() {
        printf("Toggling contact point display\n");
		p_drawer.draw.drawContactPoints = !p_drawer.draw.drawContactPoints;
		};
    events.push_back(drawContactPointsTrigger);

    UIEventTrigger drawContactNormalsTrigger;
    drawContactNormalsTrigger.type = UIEventType::OnMouseTriggered;
    drawContactNormalsTrigger.mouse = MOUSE_BUTTON_1;
    drawContactNormalsTrigger.callback = [&]() {
        printf("Toggling contact normal display\n");
        p_drawer.draw.drawContactNormals = !p_drawer.draw.drawContactNormals;
		};
    events.push_back(drawContactNormalsTrigger);

    UIEventTrigger drawContactImpulsesTrigger;
    drawContactImpulsesTrigger.type = UIEventType::OnMouseTriggered;
    drawContactImpulsesTrigger.mouse = MOUSE_BUTTON_1;
    drawContactImpulsesTrigger.callback = [&]() {
        printf("Toggling contact impulse display\n");
		p_drawer.draw.drawContactImpulses = !p_drawer.draw.drawContactImpulses;
        };
    events.push_back(drawContactImpulsesTrigger);

    UIEventTrigger drawFrictionImpulsesTrigger;
    drawFrictionImpulsesTrigger.type = UIEventType::OnMouseTriggered;
    drawFrictionImpulsesTrigger.mouse = MOUSE_BUTTON_1;
    drawFrictionImpulsesTrigger.callback = [&]() {
        printf("Toggling friction impulse display\n");
        p_drawer.draw.drawFrictionImpulses = !p_drawer.draw.drawFrictionImpulses;
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
    p_drawer.Create();
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


#pragma region Voronoi Scenes

void InitVoronoiGUI()
{
    v_drawer.Create();
}

void UpdateVoronoiGUI()
{
    if (v_scene_manager.currentScene < 0)
        v_scene_manager.ChangeScene(0);
    
    v_scene_manager.Update(CP_System_GetDt());

    if (CP_Input_KeyTriggered(KEY_EQUAL))
    {
        int index = (v_scene_manager.currentScene + 1) % v_scene_manager.sceneCount;
        v_scene_manager.ChangeScene(index);
    }
}

#pragma endregion





void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    CP_System_ShowConsole();
    InitChioriGUI();
    InitVoronoiGUI();
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
        UpdateVoronoiGUI();
    


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
