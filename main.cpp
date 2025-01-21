
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "chioriMath.h"
#include "cActor.h"
#include "cprocessing.h"
#include "physicsWorld.h"
#include "aabbtree.h"
#include "gjk.h"

using namespace chiori;
cPhysicsWorld world; // create an instance of the physics world
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
vec2 middle = vec2{ recommendedWidth / 2.0f, recommendedHeight - 100.0f };

bool IsPointInRadius(CP_Vector center, float radius, CP_Vector pos)
{
    float distX = pos.x - center.x;
    float distY = pos.y - center.y;
    float distance = (distX * distX) + (distY * distY);
    return distance < (radius * radius);
}

bool IsPointInRadius(vec2 center, float radius, vec2 pos)
{
    float distX = pos.x - center.x;
    float distY = pos.y - center.y;
    float distance = (distX * distX) + (distY * distY);
    return distance < (radius * radius);
}

void DrawActor(cShape*& inShape, const cTransform& inTfm)
{    
    cTransform xfAlt = inTfm;
    xfAlt.p.y *= -1;
    xfAlt.p *= 100;
    xfAlt.p += middle;
    xfAlt.scale = { 100,100 };
    std::vector<vec2> vertices(inShape->getCount());
    inShape->getVertices(vertices.data(), xfAlt);
    for (int i = 0; i < vertices.size(); i++)
    {
        CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
        vec2 vert = vertices[i];
        vec2 nxtVert = vertices[(i + 1) % vertices.size()];
        CP_Graphics_DrawCircle(vert.x, vert.y, 2);
        CP_Settings_StrokeWeight(1);
        CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
        CP_Graphics_DrawLine(vert.x, vert.y, nxtVert.x, nxtVert.y);
    }
    vec2 middlePos = inTfm.p * 100;
    middlePos.y *= -1;
    middlePos += middle;
    CP_Graphics_DrawCircle(middlePos.x, middlePos.y, 3);
}

void DrawContact(cContact* contact)
{
    int pointCount = contact->manifold.pointCount;
    vec2 normal = contact->manifold.normal;
    cTransform xf = world.p_actors[world.p_shapes[contact->shapeIndexA]->actorIndex]->getTransform();
    for (int j = 0; j < pointCount; ++j)
    {
        cManifoldPoint* point = contact->manifold.points + j;
        vec2 anchor = point->localAnchorA;
        
        if (anchor == vec2::zero)
            continue;
        
        anchor.y *= -1;
        vec2 worldPoint = cTransformVec(xf, anchor);
        worldPoint *= 100; worldPoint += middle;
        
        if (point->separation > 0.005f) // speculative point
        {
            CP_Settings_Stroke(CP_Color_Create(255, 255, 255, 255));
            CP_Graphics_DrawCircle(worldPoint.x, worldPoint.y, 3);
        }
        else if (!point->persisted)
        {
            CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255));
            CP_Graphics_DrawCircle(worldPoint.x, worldPoint.y, 6);
        }
        else if (point->persisted)
        {
            CP_Settings_Stroke(CP_Color_Create(0, 0, 255, 255));
            CP_Graphics_DrawCircle(worldPoint.x, worldPoint.y, 3);
        }

        vec2 normalAnchor = anchor + contact->manifold.normal;
        normalAnchor.y *= -1;
        vec2 normalCap = cTransformVec(xf, normalAnchor);
        normalCap *= 100;
        normalCap += middle;

        vec2 line = normalCap - worldPoint;
        line *= point->separation;
        normalCap = worldPoint + line;

        CP_Settings_StrokeWeight(3);
        CP_Settings_Stroke(CP_Color_Create(255, 255, 255, 255));
        CP_Graphics_DrawLine(worldPoint.x, worldPoint.y, normalCap.x, normalCap.y);
    }
}

void BoxScene()
{
    // Create a floor
    ActorConfig a_config;
    a_config.type = cActorType::STATIC;
    a_config.position = vec2{ 0, 0 };
    int floorActorIndex = world.CreateActor(a_config);
    
    ShapeConfig s_config;
    cPolygon floorShape = GeomMakeBox(5.0f, 0.25f);
    int floorShapeIndex = world.CreateShape(floorActorIndex, s_config, &floorShape);

    a_config.type = cActorType::DYNAMIC;
    a_config.position = vec2{ 1.0f, 0.5f };
    a_config.rotation =  35 * DEG2RAD;
    int boxAActorIndex = world.CreateActor(a_config);
    cPolygon boxAShape = GeomMakeBox(0.5f, 0.5f);
    int boxAShapeIndex = world.CreateShape(boxAActorIndex, s_config, &boxAShape);
}

void InitPhysics()
{
    BoxScene();
}



void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    CP_System_ShowConsole();
	InitPhysics();
}

bool pauseStep = false;
bool isPaused = true;
void RunChioriPhysics()
{
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

    for (int itr = 0; itr < world.p_actors.size(); itr++)
    {
        cActor* a = world.p_actors[itr];
        int shapeIndex = a->shapeList;
        while (shapeIndex != -1)
        {
            cShape* s = world.p_shapes[shapeIndex];
            DrawActor(s, a->getTransform());
            shapeIndex = s->nextShapeIndex;
        }
    }
    for (int itr = 0; itr < world.p_contacts.size(); itr++)
    {
        cContact* c = world.p_contacts[itr];
        DrawContact(c);
    }

    auto drawFunc = [&](int height, const AABB& aabb)
        {
            CP_Settings_StrokeWeight(2);
            CP_Settings_Stroke(CP_Color_Create(50, 50, 255, 255));
            vec2 aabbv[4];
            aabbv[0] = { aabb.min };
            aabbv[1] = { aabb.max.x, aabb.min.y };
            aabbv[2] = { aabb.max };
            aabbv[3] = { aabb.min.x, aabb.max.y };
            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                vec2 p = aabbv[i];
                vec2 q = aabbv[j];
                p.y = -p.y;
                q.y = -q.y;
                p *= 100;
                q *= 100;
                p += middle;
                q += middle;

                CP_Graphics_DrawLine(p.x, p.y, q.x, q.y);
            }
            CP_Settings_Fill(CP_Color_Create(127, 127, 255, 255));
        };
    world.m_broadphase.GetTree().DisplayTree(drawFunc);
    
}

cActor* selectedActor;
void HandleInput(CP_Vector mousePosIn)
{
    //std::cout << "Is Holding: " << ((isHolding_mouse) ? selectedActor->shapeIndex : -1) << std::endl;
    CP_Vector mousePos = mousePosIn;// CP_Vector_Add(mousePosIn, { middle.x / 2, middle.y / 2 });
    vec2 worldPos = { mousePos.x, mousePos.y };
    worldPos -= middle;
    worldPos /= 100;
    worldPos.y = -worldPos.y;
    
    if ((CP_Input_MouseDown(MOUSE_BUTTON_1) && !isHolding_mouse))
    {
        for (int itr = 0; itr < world.p_actors.size(); itr++)
        {
            cActor* a = world.p_actors[itr];
            if (a->type == cActorType::STATIC)
                continue;
            cTransform tfm = a->getTransform();
            if (IsPointInRadius(tfm.p, 1, worldPos))
            {
                selectedActor = a;
                isHolding_mouse = true;
                break;
            }
        }
    }
    else if (CP_Input_KeyTriggered(KEY_P))
    {
        if (!isHolding_keys)
        {
            for (int itr = 0; itr < world.p_actors.size(); itr++)
            {
                cActor* a = world.p_actors[itr];
                cTransform tfm = a->getTransform();
                if (IsPointInRadius(tfm.p, 1, worldPos))
                {
                    selectedActor = a;
                    isHolding_keys = true;
                    break;
                }
            }
        }
        else
        {
            isHolding_keys = false;
            selectedActor = nullptr;
        }
    }

    if (CP_Input_MouseReleased(MOUSE_BUTTON_1))
    {
        selectedActor = nullptr;
        isHolding_mouse = false;
    }

    if (isHolding_mouse && selectedActor)
    {
        selectedActor->linearVelocity = vec2::zero;
        cTransform tfm = selectedActor->getTransform();
        tfm.p = { worldPos.x, worldPos.y };
        selectedActor->setTransform(tfm);

        //if (CP_Input_MouseDown(MOUSE_BUTTON_2))
        //{
        //    cTransform tfm = selectedActor->getTransform();
        //    tfm.q += (35 * DEG2RAD) * CP_System_GetDt();
        //    selectedActor->setTransform(tfm);
        //}

        if (CP_Input_KeyTriggered(KEY_K))
        {
            selectedActor->addTorque(5000.0f);
        }
    }
    
    if (isHolding_keys && selectedActor)
    {
        cTransform tfm = selectedActor->getTransform();
        vec2& pos = tfm.p;

        if (CP_Input_KeyDown(KEY_RIGHT))
        {
            tfm.p += vec2::right * 0.1f;
        }
        else if (CP_Input_KeyDown(KEY_LEFT))
        {
            tfm.p += vec2::left * 0.1f;
        }
        else if (CP_Input_KeyDown(KEY_UP))
        {
            tfm.p += vec2::down * 0.1f;
        }
        else if (CP_Input_KeyDown(KEY_DOWN))
        {
            tfm.p += vec2::up * 0.1f;
        }
        selectedActor->setTransform(tfm);
    }
}

void game_update(void)
{
    CP_Settings_BlendMode(CP_BLEND_ALPHA);
    CP_Graphics_ClearBackground(CP_Color_Create(51, 51, 51, 255));
    CP_Settings_Fill(CP_Color_Create(0, 0, 0, 255));
    
    CP_Vector mousePos = CP_Vector{ (float)CP_Input_GetMouseWorldX(), (float)CP_Input_GetMouseWorldY() };
    HandleInput(mousePos);
    RunChioriPhysics();

    // Profiling info and frameRate testing
    if (drawFPS)
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
