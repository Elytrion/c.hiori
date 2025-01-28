
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "cprocessing.h"
#include "graphics.h"
#include "uimanager.h"
#include "physicsWorld.h"

using namespace chiori;


cPhysicsWorld world; // create an instance of the physics world
DebugGraphics drawer; // create a graphics instance to draw the world and UI
UIManager ui_manager{ &drawer }; // create a ui manager to handle UI input events

bool isHolding_mouse = false;
bool isHolding_keys = false;
CP_Color randomColors[] = {
    { 128, 0,   0,   255 },
    { 128, 128, 0,   255 },
    { 0,   128, 0,   255 },
    { 0,   128, 128, 255 },
    { 0,   0,   128, 255 },
    { 128, 0,   128, 255 } };
float recommendedWidth = 1600.0f;
float recommendedHeight = 900.0f;
bool drawColors = true;
bool drawFPS = true;
cVec2 middle = cVec2{ recommendedWidth / 2.0f, recommendedHeight - 100.0f };
float scaleValue = 50.0f;

bool IsPointInRadius(CP_Vector center, float radius, CP_Vector pos)
{
    float distX = pos.x - center.x;
    float distY = pos.y - center.y;
    float distance = (distX * distX) + (distY * distY);
    return distance < (radius * radius);
}

bool IsPointInRadius(cVec2 center, float radius, cVec2 pos)
{
    float distX = pos.x - center.x;
    float distY = pos.y - center.y;
    float distance = (distX * distX) + (distY * distY);
    return distance < (radius * radius);
}

void RampScene()
{
    // Create a floor
    ActorConfig a_config;
    a_config.type = cActorType::STATIC;
    int staticID = world.CreateActor(a_config);

    a_config.position = { 0.0f, 10.0f };
    a_config.angle = -0.25f;
    int staticID2 = world.CreateActor(a_config);

    ShapeConfig s_config;
    s_config.friction = 0.2f;
    cPolygon floorShape = GeomMakeBox(15.0f, 0.25f);
    int floorShapeIndex = world.CreateShape(staticID, s_config, &floorShape);

    cPolygon ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -3.0f, 12.0f }, -0.25f);
    world.CreateShape(staticID, s_config, &ramp);
    
    ramp = GeomMakeOffsetBox(0.1f, 0.75f, { 4.5f, 10.3f });
    world.CreateShape(staticID, s_config, &ramp);

    ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -1.0f, 7.0f }, 0.25f);
    world.CreateShape(staticID, s_config, &ramp);

    ramp = GeomMakeOffsetBox(0.1f, 0.75f, { -8.3f, 5.5f });
    world.CreateShape(staticID, s_config, &ramp);
    
    ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -3.0f, 2.0f }, -0.25f);
    world.CreateShape(staticID, s_config, &ramp);

    cPolygon box = GeomMakeBox(0.3f, 0.3f);
    s_config.density = 25.0f;
    float friction[5] = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

    for (int i = 0; i < 5; ++i)
    {
        ActorConfig box_config;
        box_config.type = cActorType::DYNAMIC;
        box_config.position = { -8.0f + 2.0f * i, 14.0f };
        int bodyId = world.CreateActor(box_config);

        s_config.friction = friction[i];
        world.CreateShape(bodyId, s_config, &box);
    }


    {
        scaleValue = 45.0f;
        drawer.ChangeZoom(scaleValue);
		drawer.SetCamera({ 0.0f, 7.0f });
    }
}


void InitPhysics()
{
    //BoxScene();
    RampScene();

    drawer.Create();
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

void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    CP_System_ShowConsole();
	InitPhysics();
    InitUI();
}



cActor* selectedActor;
void HandleInput(CP_Vector mousePosIn)
{
    if (CP_Input_KeyDown(KEY_LEFT_BRACKET))
    {
        scaleValue = c_clamp(scaleValue - 0.5f, 1.0f, 1000.0f);
        drawer.ChangeZoom(scaleValue);
    }
    else if (CP_Input_KeyDown(KEY_RIGHT_BRACKET))
    {
        scaleValue = c_clamp(scaleValue + 0.5f, 1.0f, 1000.0f);
        drawer.ChangeZoom(scaleValue);
    }

    cVec2 moveDelta = cVec2::zero;
    bool moved = false;
    if (CP_Input_KeyDown(KEY_DOWN))
    {
        moveDelta.y -= 5.0f;
        moved = true;
    }
    else if (CP_Input_KeyDown(KEY_UP))
    {
        moveDelta.y += 5.0f;
        moved = true;
    }

    if (CP_Input_KeyDown(KEY_LEFT))
    {
        moveDelta.x -= 5.0f;
        moved = true;
    }
    else if (CP_Input_KeyDown(KEY_RIGHT))
    {
        moveDelta.x += 5.0f;
        moved = true;
    }

    if (moved)
    {
        drawer.PanCamera(moveDelta);
    }
}

bool pauseStep = false;
bool isPaused = true;
void game_update(void)
{
    CP_Settings_BlendMode(CP_BLEND_ALPHA);
    CP_Graphics_ClearBackground(CP_Color_Create(51, 51, 51, 255));
    CP_Settings_Fill(CP_Color_Create(0, 0, 0, 255));
    
    CP_Vector mousePos = CP_Vector{ (float)CP_Input_GetMouseWorldX(), (float)CP_Input_GetMouseWorldY() };
    HandleInput(mousePos);
    
    if (!isPaused)
    {
        world.update(CP_System_GetDt());
        if (CP_Input_KeyTriggered(KEY_SPACE))
            isPaused = true;
    }
    else
    {
        if (CP_Input_KeyTriggered(KEY_SPACE))
            isPaused = false;

        if (CP_Input_KeyTriggered(KEY_SLASH) || CP_Input_KeyDown(KEY_PERIOD))
            pauseStep = true;

        if (pauseStep)
        {
            world.step(1.0f / 60.0f);
            pauseStep = false;
        }
    }

    ui_manager.Update();
    drawer.DrawFrame(&world);

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
