#include "pch.h"
#include "scenemanager.h"
#include "physicsWorld.h"
#include "graphics.h"
#include "cprocessing.h"

using namespace chiori;

void PhysicsScene::Unload()
{
    cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);
    int capacity = pWorld->p_actors.capacity();
    for (int i = capacity - 1; i >= 0; i--)
    {
        if (pWorld->p_actors.isValid(i))
            pWorld->RemoveActor(i);
    }
}

void PhysicsScene::Update(float dt)
{
	accumulator += dt;
	const float maxAccumulator = settings.physicsStepTime * 5;
	if (accumulator > maxAccumulator)
	{
		accumulator = maxAccumulator;
	}

    while (accumulator >= settings.physicsStepTime)
    {
        // Call the physics step function with the fixed time step
        Step(settings.physicsStepTime);
        // Subtract the fixed step time from the accumulator
        accumulator -= settings.physicsStepTime;
    }
}

void PhysicsScene::Step(float fdt)
{
    cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);
    pWorld->runBasicSolver = settings.runBasicSolver;
    pWorld->step(
        settings.physicsStepTime,
        settings.primaryIterations,
        settings.secondaryIterations,
        settings.warmStart
    );
}

#pragma region Scenes
// loads in a default scene of 2 stacked boxes and a floor
class DefaultScene : public PhysicsScene
{
public:
    DefaultScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

        // Create a floor
        ActorConfig a_config;
        a_config.type = cActorType::STATIC;
        int floorID = pWorld->CreateActor(a_config);

        ShapeConfig s_config;
        s_config.friction = 0.2f;
        cPolygon floorShape = GeomMakeBox(10.0f, 0.25f);
        int floorShapeIndex = pWorld->CreateShape(floorID, s_config, &floorShape);

        // Create the ground box
        a_config.type = cActorType::DYNAMIC;
        a_config.position = { 0.0f, 1.0f };
        int boxID = pWorld->CreateActor(a_config);
        cPolygon box = GeomMakeBox(1.0f, 1.0f);
        int boxShapeIndex = pWorld->CreateShape(boxID, s_config, &box);

        // Create the top box
        a_config.position = { 0.25f, 3.5f };
        int boxID2 = pWorld->CreateActor(a_config);
        int boxShapeIndex2 = pWorld->CreateShape(boxID2, s_config, &box);

        // configure camera
        {
            currentZoom = 80.0f;
            drawer->ChangeZoom(currentZoom);
            drawer->SetCamera({ 0.0f, 5.0f });
        }
    }
};

class RampScene : public PhysicsScene
{
public:
    RampScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

        // Create a floor
        ActorConfig a_config;
        a_config.type = cActorType::STATIC;
        int staticID = pWorld->CreateActor(a_config);

        a_config.position = { 0.0f, 10.0f };
        a_config.angle = -0.25f;
        int staticID2 = pWorld->CreateActor(a_config);

        ShapeConfig s_config;
        s_config.friction = 0.2f;
        cPolygon floorShape = GeomMakeBox(15.0f, 0.25f);
        int floorShapeIndex = pWorld->CreateShape(staticID, s_config, &floorShape);

        cPolygon ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -3.0f, 12.0f }, -0.25f);
        pWorld->CreateShape(staticID, s_config, &ramp);

        ramp = GeomMakeOffsetBox(0.1f, 0.75f, { 4.5f, 10.3f });
        pWorld->CreateShape(staticID, s_config, &ramp);

        ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -1.0f, 7.0f }, 0.25f);
        pWorld->CreateShape(staticID, s_config, &ramp);

        ramp = GeomMakeOffsetBox(0.1f, 0.75f, { -8.3f, 5.5f });
        pWorld->CreateShape(staticID, s_config, &ramp);

        ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -3.0f, 2.0f }, -0.25f);
        pWorld->CreateShape(staticID, s_config, &ramp);

        cPolygon box = GeomMakeBox(0.3f, 0.3f);
        s_config.density = 25.0f;
        float friction[5] = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

        for (int i = 0; i < 5; ++i)
        {
            ActorConfig box_config;
            box_config.type = cActorType::DYNAMIC;
            box_config.position = { -8.0f + 2.0f * i, 14.0f };
            int bodyId = pWorld->CreateActor(box_config);

            s_config.friction = friction[i];
            pWorld->CreateShape(bodyId, s_config, &box);
        }


        {
            currentZoom = 45.0f;
            drawer->ChangeZoom(currentZoom);
            drawer->SetCamera({ 0.0f, 7.0f });
        }
    }
};
#pragma endregion

SceneManager::SceneManager(DebugGraphics* drawer, void* world)
    : drawer(drawer), world(world)
{
    AddScene(new DefaultScene(drawer, world));
    AddScene(new RampScene(drawer, world));
	ChangeScene(0);
}

SceneManager::~SceneManager()
{
    for (int i = scenes.size() - 1; i >= 0; i--)
    {
		delete scenes[i];
	}
}

void SceneManager::ChangeScene(int sceneIndex)
{
    bool prevPaused = isPaused;
    isPaused = true;
    if (currentScene >= 0)
    {
		scenes[currentScene]->Unload();
	}
    currentScene = sceneIndex;
	scenes[currentScene]->Load();
    isPaused = prevPaused;
}

void SceneManager::Update(float dt)
{
    if (!isPaused)
    {
        scenes[currentScene]->Update(CP_System_GetDt());
        if (CP_Input_KeyTriggered(KEY_SPACE))
            isPaused = true;
    }
    else
    {
        if (CP_Input_KeyTriggered(KEY_SPACE))
            isPaused = false;

        if (CP_Input_KeyTriggered(KEY_SLASH) || CP_Input_KeyDown(KEY_PERIOD))
            stepOnce = true;

        if (stepOnce)
        {
            scenes[currentScene]->Step(scenes[currentScene]->settings.physicsStepTime);
            stepOnce = false;
        }
    }

    if (CP_Input_KeyTriggered(KEY_I))
    {
        drawInstructions = !drawInstructions;
    }
    if (CP_Input_KeyTriggered(KEY_C))
    {
		drawCamera = !drawCamera;
	}

    HandleCameraInput();
    drawer->DrawFrame(world);
    cVec2 displayDim = drawer->getScreenDimensions();
    // draw camera
    if (drawCamera)
    {
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

        char buffer[64];
        CP_Color textColor = CP_Color{ 50, 255, 50, 255 };
        snprintf(buffer, 64, "Cam World Pos: (%.2f, %.2f)", drawer->getCameraWorldPos().x, drawer->getCameraWorldPos().y);
        drawer->DrawUIText(displayDim.x - 200, displayDim.y - 20, buffer, 15, textColor);

        // draw 2 lines +x +y from camera pos
        cVec2 camScreenPos = drawer->getCameraScreenPos();
        cVec2 xLineEnd = camScreenPos + cVec2::right * 10.0f;
        cVec2 yLineEnd = camScreenPos + -cVec2::up * 10.0f;
        drawer->DrawUILine(camScreenPos.x, camScreenPos.y, xLineEnd.x, xLineEnd.y, CP_Color{ 255, 0, 100, 255 });
        drawer->DrawUILine(camScreenPos.x, camScreenPos.y, yLineEnd.x, yLineEnd.y, CP_Color{ 100, 255, 0, 255 });
    }

    if (drawInstructions)
    {
        char buffer[64];
        CP_Color textColor = CP_Color{ 50, 255, 50, 255 };
        snprintf(buffer, 64, "Press SPACE to %s", isPaused ? "Play" : "Pause");
        drawer->DrawUIText(displayDim.x - 220, 20, buffer, 15, textColor);

        snprintf(buffer, 64, "Press SLASH to step once");
        drawer->DrawUIText(displayDim.x - 220, 40, buffer, 15, textColor);
        
        snprintf(buffer, 64, "Hold PERIOD to keep stepping");
        drawer->DrawUIText(displayDim.x - 220, 60, buffer, 15, textColor);

        snprintf(buffer, 64, "Press EQUAL to change scenes");
        drawer->DrawUIText(displayDim.x - 220, 80, buffer, 15, textColor);

        snprintf(buffer, 64, "Press LEFT/RIGHT BRACKET to zoom");
        drawer->DrawUIText(displayDim.x - 220, 100, buffer, 15, textColor);

        snprintf(buffer, 64, "Press UP/DOWN/LEFT/RIGHT to pan");
        drawer->DrawUIText(displayDim.x - 220, 120, buffer, 15, textColor);

        snprintf(buffer, 64, "Press I to toggle instructions");
        drawer->DrawUIText(displayDim.x - 220, 140, buffer, 15, textColor);

        snprintf(buffer, 64, "Press C to toggle camera display");
        drawer->DrawUIText(displayDim.x - 220, 160, buffer, 15, textColor);
    }

    if (drawStats)
    {

    }
}

void SceneManager::HandleCameraInput()
{
    float& camZoom = scenes[currentScene]->currentZoom;
    if (CP_Input_KeyDown(KEY_LEFT_BRACKET))
    {
        camZoom = c_clamp(camZoom - 0.5f, 1.0f, 1000.0f);
        drawer->ChangeZoom(camZoom);
    }
    else if (CP_Input_KeyDown(KEY_RIGHT_BRACKET))
    {
        camZoom = c_clamp(camZoom + 0.5f, 1.0f, 1000.0f);
        drawer->ChangeZoom(camZoom);
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
        drawer->PanCamera(moveDelta);
    }
}

void SceneManager::AddScene(PhysicsScene* scene)
{
    sceneCount++;
    scenes.push_back(scene);
}

void SceneManager::RemoveScene(int sceneIndex)
{
    if (sceneIndex >= 0 && sceneIndex < sceneCount)
    {
		delete scenes[sceneIndex];
		scenes.erase(scenes.begin() + sceneIndex);
		sceneCount--;
	}
}

PhysicsScene* SceneManager::GetCurrentScene()
{
	return scenes[currentScene];
}