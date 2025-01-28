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
// Shows basic physics simulation with collision and gravity
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
// Loads in a scene with slanted ramps and 5 boxes of varying friction
// Boxes with high friction should be able to stop on ramped surfaces
// Boxes with low friction should slide down the ramps
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

// Loads in a scene with a row of 15 dominos, with the first one set to fall
// The dominos should knock each other over, then at the end, 
// the last domino should fall in a way to push all the previous dominos flat (double-domino effect)
class DominoScene : public PhysicsScene
{
public:
    DominoScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
		cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

		// Create a floor
		ActorConfig a_config;
        a_config.position = { 0.0f, -1.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		cPolygon floorShape = GeomMakeBox(20.0f, 1.0f);
		pWorld->CreateShape(staticID, s_config, &floorShape);
	    
        // Create row of dominos
        cPolygon box = GeomMakeBox(0.125f, 0.5f);
        s_config.friction = 0.6f;
        a_config.type = cActorType::DYNAMIC;

        int count = 17;
        float x = -0.5f * count;
        for (int i = 0; i < count; ++i)
        {
            a_config.position = { x, 0.5f };
            int bodyId = pWorld->CreateActor(a_config);
            pWorld->CreateShape(bodyId, s_config, &box);
            if (i == 0)
            {
                cActor* actor = pWorld->p_actors[bodyId];
                actor->applyImpulse({ 0.2f, 0.0f }, { x, 1.0f });
            }

            x += 1.0f;
        }

        {
            currentZoom = 75.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 4.0f });
        }
    }
};

// Loads in a scene of a pyramid of boxes that are intially overlapping
// The scene should recover from the overlap and stabilize
class OverlapRecoveryScene : public PhysicsScene
{
public:
    OverlapRecoveryScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

        // Create a floor
        ActorConfig a_config;
        a_config.position = { 0.0f, -1.0f };
        a_config.type = cActorType::STATIC;
        int staticID = pWorld->CreateActor(a_config);
        ShapeConfig s_config;
        cPolygon floorShape = GeomMakeBox(10.0f, 1.0f);
        pWorld->CreateShape(staticID, s_config, &floorShape);


        int baseCount = 4;
        float overlap = 0.25f;
        float extent = 0.5f;
        float fraction = 1.0f - overlap;
        float y = 0.3f;

        a_config.type = cActorType::DYNAMIC;
        cPolygon box = GeomMakeBox(extent, extent);

        for (int i = 0; i < baseCount; ++i)
        {
            float x = fraction * extent * (i - baseCount);
            for (int j = i; j < baseCount; ++j)
            {
                a_config.position = { x, y };
                int bodyId = pWorld->CreateActor(a_config);

                pWorld->CreateShape(bodyId, s_config, &box);

                x += 2.0f * fraction * extent;
            }

            y += 2.0f * fraction * extent;
        }
    }
};

// Loads in a scene with a stack of 6 boxes on one side
// and a Tower of Lire of 6 boxes on the other
// The scene should demonstrate the stability of the physics simulation
class StackScene : public PhysicsScene
{
public:
    StackScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

        // Create a floor
        ActorConfig a_config;
        a_config.position = { 0.0f, -1.0f };
        a_config.type = cActorType::STATIC;
        int staticID = pWorld->CreateActor(a_config);

        ShapeConfig s_config;
        cPolygon floorShape = GeomMakeBox(20.0f, 1.0f);
        pWorld->CreateShape(staticID, s_config, &floorShape);

        // Create the stack of 5 boxes on one side
        cPolygon boxShape = GeomMakeBox(0.5f, 0.5f); // Box dimensions: 1x1
        s_config.friction = 0.5f;
        int boxCount = 6;
        a_config.type = cActorType::DYNAMIC;
        float xStack = -5.0f; // Position of the stack
        float yStack = 1.0f;  // Start from the ground

        for (int i = 0; i < boxCount; ++i)
        {
            a_config.position = { xStack, yStack };
            int bodyId = pWorld->CreateActor(a_config);
            pWorld->CreateShape(bodyId, s_config, &boxShape);

            yStack += 1.0f; // Stack each box 1 unit higher
        }

        // Create the tower of Lire (block-stacking problem) on the other side
        float xTower = 5.0f; // Position of the tower
        float yTower = 0.0f + 1.0f * boxCount; // Start from the top

        for (int i = 0; i < boxCount; ++i)
        {
            float offset = 1.0f / (2.0f * (i + 1)); // Offset for overhang, largest at the top

            a_config.position = { xTower + offset, yTower };
            int bodyId = pWorld->CreateActor(a_config);
            pWorld->CreateShape(bodyId, s_config, &boxShape);

            yTower -= 1.0f; // Raise the next box
        }

        // Configure the camera to view both setups
        {
            currentZoom = 50.0f; // Adjust zoom to fit both setups
            drawer->ChangeZoom(currentZoom);

            // Center the camera between the two setups, with a slight upward adjustment
            drawer->SetCamera({ 0.0f, 2.5f });
        }
    }
};

// Loads in a scene with an arch held up by friction and their contact points
// The scene should further demonstrate the stability of the physics simulation
class ArchScene : public PhysicsScene
{
public:
    ArchScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

        // Configure camera
        {
            currentZoom = 35.0f;
            drawer->ChangeZoom(currentZoom);
            drawer->SetCamera({ 0.0f, 8.0f });
        }

        // Define polygon points for the arch
        cVec2 ps1[9] = { {16.0f, 0.0f},
                        {14.93803712795643f, 5.133601056842984f},
                        {13.79871746027416f, 10.24928069555078f},
                        {12.56252963284711f, 15.34107019122473f},
                        {11.20040987372525f, 20.39856541571217f},
                        {9.66521217819836f, 25.40369899225096f},
                        {7.87179930638133f, 30.3179337000085f},
                        {5.635199558196225f, 35.03820717801641f},
                        {2.405937953536585f, 39.09554102558315f} };

        cVec2 ps2[9] = { {24.0f, 0.0f},
                        {22.33619528222415f, 6.02299846205841f},
                        {20.54936888969905f, 12.00964361211476f},
                        {18.60854610798073f, 17.9470321677465f},
                        {16.46769273811807f, 23.81367936585418f},
                        {14.05325025774858f, 29.57079353071012f},
                        {11.23551045834022f, 35.13775818285372f},
                        {7.752568160730571f, 40.30450679009583f},
                        {3.016931552701656f, 44.28891593799322f} };

        // Scale the points
        float scale = 0.25f;
        for (int i = 0; i < 9; ++i)
        {
            ps1[i] = ps1[i] * scale;
            ps2[i] = ps2[i] * scale;
        }


        // Create a floor
        ActorConfig a_config;
        a_config.position = { 0.0f, -2.0f };
        a_config.type = cActorType::STATIC;
        int staticID = pWorld->CreateActor(a_config);

        ShapeConfig s_config;
        s_config.friction = 0.6f;
        cPolygon floorShape = GeomMakeBox(100.0f, 1.0f);
        pWorld->CreateShape(staticID, s_config, &floorShape);

        a_config.position = { 0.0f, 0.0f };
        // Create left side of the arch
        ActorConfig dynamicConfig;
        dynamicConfig.type = cActorType::DYNAMIC;
        for (int i = 0; i < 8; ++i)
        {
            cVec2 ps[4] = { ps1[i], ps2[i], ps2[i + 1], ps1[i + 1] };
            cPolygon polygon{ ps, 4 };
            dynamicConfig.position = { 0.0f, 0.0f };
            int bodyID = pWorld->CreateActor(dynamicConfig);

            pWorld->CreateShape(bodyID, s_config, &polygon);
        }

        // Create right side of the arch
        for (int i = 0; i < 8; ++i)
        {
            cVec2 ps[4] = {
                {-ps2[i].x, ps2[i].y}, {-ps1[i].x, ps1[i].y}, {-ps1[i + 1].x, ps1[i + 1].y}, {-ps2[i + 1].x, ps2[i + 1].y} };
            cPolygon polygon{ ps, 4 };
            dynamicConfig.position = { 0.0f, 0.0f };
            int bodyID = pWorld->CreateActor(dynamicConfig);

            pWorld->CreateShape(bodyID, s_config, &polygon);
        }

        // Create the top of the arch
        {
            cVec2 ps[4] = { ps1[8], ps2[8], {-ps2[8].x, ps2[8].y}, {-ps1[8].x, ps1[8].y} };
            cPolygon polygon{ ps, 4 };
            dynamicConfig.position = { 0.0f, 0.0f };
            int bodyID = pWorld->CreateActor(dynamicConfig);

            pWorld->CreateShape(bodyID, s_config, &polygon);
        }

    }
};

// Loads in a scene with a heap of polygons of varying sizes
// Pressing R will apply a random force to a random polygon
// This showcases the engine can support more complex shapes
class PolygonScene : public PhysicsScene
{
public:
    PolygonScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

    void Load() override
    {
		cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);

		// Create a confining box
		ActorConfig a_config;
		a_config.position = { 0.0f, -1.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		cPolygon wallShape = GeomMakeOffsetBox( 15.0f, 0.25f, { 0.0f, 0.0f });
        pWorld->CreateShape(staticID, s_config, &wallShape);

        wallShape = GeomMakeOffsetBox(15.0f, 0.25f, { 0.0f, 30.0f });
        pWorld->CreateShape(staticID, s_config, &wallShape);

        wallShape = GeomMakeOffsetBox(0.25f, 15.0f, { 15.25f, 15.0f });
        pWorld->CreateShape(staticID, s_config, &wallShape);

        wallShape = GeomMakeOffsetBox(0.25f, 15.0f, { -15.25f, 15.0f });
        pWorld->CreateShape(staticID, s_config, &wallShape);


        a_config.type = cActorType::DYNAMIC;
        // create a heap of polygons
        for (int iy = 1; iy < 5; ++iy)
        {
            for (int i = 3; i < 8; ++i)
            {
                cPolygon poly = GeomMakeRegularPolygon(i);

                a_config.position = { -5.0f + 1.0f * i, iy * 2.0f };

                int bodyID = pWorld->CreateActor(a_config);
                pWorld->CreateShape(bodyID, s_config, &poly);
            }
        }
	}

    void Update(float dt) override
    {
        PhysicsScene::Update(dt);

        // Pick a random polygon and apply a random force
        cPhysicsWorld* pWorld = static_cast<cPhysicsWorld*>(world);
        if (CP_Input_KeyTriggered(KEY_R))
        {
            int bodyID = -1;
            while (!pWorld->p_actors.isValid(bodyID))
            {
                bodyID = CP_Random_RangeInt(0, pWorld->p_actors.capacity() - 1);
            }
			cActor* actor = pWorld->p_actors[bodyID];

            float xforce = CP_Random_RangeFloat(-100.0f, 100.0f);
            float yforce = CP_Random_RangeFloat(-100.0f, 100.0f);

			cVec2 force = { xforce, yforce };
			actor->applyImpulse(force, actor->position);
		}
    }

};
#pragma endregion

#pragma region Scene Manager
SceneManager::SceneManager(DebugGraphics* drawer, void* world)
    : drawer(drawer), world(world)
{

    AddScene(new DefaultScene(drawer, world));
    AddScene(new StackScene(drawer, world));
    AddScene(new RampScene(drawer, world));
    AddScene(new DominoScene(drawer, world));
    AddScene(new ArchScene(drawer, world));
    AddScene(new OverlapRecoveryScene(drawer, world));
    AddScene(new PolygonScene(drawer, world));

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
    if (CP_Input_KeyTriggered(KEY_S))
    {
        drawStats = !drawStats;
    }
    if (CP_Input_KeyTriggered(KEY_F1))
    {
        if (!isPaused)
        {
            std::cout << "Cannot change solvers while unpaused!\n" << std::endl;
        }
        else
        {
            scenes[currentScene]->settings.runBasicSolver = !scenes[currentScene]->settings.runBasicSolver;
        }
	}
    if (CP_Input_KeyTriggered(KEY_ESCAPE))
    {
        // reload the current scene
        ChangeScene(currentScene);
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
        float x = displayDim.x - 220;
        float y = 20;
        snprintf(buffer, 64, "Press SPACE to %s", isPaused ? "Play" : "Pause");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press SLASH to step once");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Hold PERIOD to keep stepping");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press EQUAL to change scenes");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press ESC to reset scene");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press LEFT/RIGHT BRACKET to zoom");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press UP/DOWN/LEFT/RIGHT to pan");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press I to toggle instructions");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press C to toggle camera display");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press S to toggle stats");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;

        snprintf(buffer, 64, "Press F1 to toggle solvers");
        drawer->DrawUIText(x, y, buffer, 15, textColor);
        y += 20;
    }

    if (drawStats)
    {
        char buffer[64];
        CP_Color textColor = CP_Color{ 255, 50, 50, 255 };
        snprintf(buffer, 64, "Running %s", scenes[currentScene]->settings.runBasicSolver ? "PGS Basic" : "PGS Soft");
        drawer->DrawUIText(20, displayDim.y - 20, buffer, 15, textColor);

        int actorCapacity = static_cast<cPhysicsWorld*>(world)->p_actors.capacity();
        int actorCount = static_cast<cPhysicsWorld*>(world)->p_actors.size();
        snprintf(buffer, 64, "Actors: %d/%d", actorCount, actorCapacity);
        drawer->DrawUIText(20, displayDim.y - 40, buffer, 15, textColor);

        int shapeCapacity = static_cast<cPhysicsWorld*>(world)->p_shapes.capacity();
        int shapeCount = static_cast<cPhysicsWorld*>(world)->p_shapes.size();
        snprintf(buffer, 64, "Shapes: %d/%d", shapeCount, shapeCapacity);
        drawer->DrawUIText(20, displayDim.y - 60, buffer, 15, textColor);

        CP_Settings_TextSize(20);
        CP_Settings_BlendMode(CP_BLEND_ALPHA);
        CP_Settings_Fill(CP_Color_Create(0, 0, 0, 128));
        CP_Settings_NoStroke();
        CP_Graphics_DrawRect(0, 0, 150, 30);
        CP_Settings_Fill(CP_Color_Create(255, 255, 255, 255));
        char nbuffer[100];
        sprintf_s(nbuffer, 100, "FPS: %f", CP_System_GetFrameRate());
        CP_Font_DrawText(nbuffer, 20, 20);
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
#pragma endregion