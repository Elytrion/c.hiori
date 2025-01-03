
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

std::vector<int> actors;
std::vector<int> shapes;

CP_Color randomColors[] = {
    { 128, 0,   0,   255 },
    { 128, 128, 0,   255 },
    { 0,   128, 0,   255 },
    { 0,   128, 128, 255 },
    { 0,   0,   128, 255 },
    { 128, 0,   128, 255 } };

int recommendedWidth = 1600;
int recommendedHeight = 900;
bool drawColors = true;
bool drawFPS = true;
vec2 middle = vec2{ recommendedWidth / 2.0f, recommendedHeight / 2.0f };

bool IsPointInRadius(CP_Vector center, float radius, CP_Vector pos)
{
    float distX = pos.x - center.x;
    float distY = pos.y - center.y;
    float distance = (distX * distX) + (distY * distY);
    return distance < (radius * radius);
}

void DrawActor(cShape*& inShape, const cTransform& inTfm)
{    
    cTransform xfAlt = inTfm;
    xfAlt.pos *= 100;
    xfAlt.pos += middle;
    xfAlt.scale = { 100,100 };
	const std::vector<vec2> vertices = inShape->getVertices(xfAlt);
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
    vec2 middlePos = inTfm.pos * 100;
    middlePos += middle;
    CP_Graphics_DrawCircle(middlePos.x, middlePos.y, 3);
}

void DrawContact(cContact* contact)
{
    vec2 pointA, pointB;
    int& indexA = contact->shapeIndexA;
    cTransform xf = world.p_actors[world.p_shapes[indexA]->actorIndex]->getTransform();
    vec2 anchor0 = contact->manifold.points[0].localAnchorA;
    vec2 anchor1 = contact->manifold.points[1].localAnchorA;
    pointA = cTransformVec(xf, anchor0);
    pointB = cTransformVec(xf, anchor1);
    pointA *= 100;
    pointB *= 100;
    pointA += middle;
    pointB += middle;
    CP_Graphics_DrawCircle(pointA.x, pointA.y, 3);
    CP_Graphics_DrawCircle(pointB.x, pointB.y, 3);

    vec2 pointA1, pointB1;
    vec2 normalAnchor0 = anchor0 + contact->manifold.normal;// *contact->manifold.points[0].separation;
    vec2 normalAnchor1 = anchor1 + contact->manifold.normal;// *contact->manifold.points[1].separation;

    pointA1 = cTransformVec(xf, normalAnchor0);
    pointB1 = cTransformVec(xf, normalAnchor1);
    pointA1 *= 100;
    pointB1 *= 100;
    pointA1 += middle;
    pointB1 += middle;

    vec2 lineA = pointA1 - pointA;
    lineA *= contact->manifold.points[0].separation;
    pointA1 = pointA + lineA;

    vec2 lineB = pointB1 - pointB;
    lineB *= contact->manifold.points[1].separation;
    pointB1 = pointB + lineB;

    CP_Settings_StrokeWeight(2);
    CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255));
    CP_Graphics_DrawLine(pointA.x, pointA.y, pointA1.x, pointA1.y);
    CP_Settings_Stroke(CP_Color_Create(0, 0, 255, 255));
    CP_Graphics_DrawLine(pointB.x, pointB.y, pointB1.x, pointB1.y);
}

cActor* CreateRectActor(float width, float height, vec2& centrePos)
{
    std::vector<vec2> vertices;
	vertices.push_back({ -width / 2.0f, -height / 2.0f });
	vertices.push_back({ width / 2.0f, -height / 2.0f });
	vertices.push_back({ width / 2.0f, height / 2.0f });
	vertices.push_back({ -width / 2.0f, height / 2.0f });

    ActorConfig a_config;
    a_config.position = centrePos;
    int newActorIndex = world.CreateActor(a_config);
    ShapeConfig s_config;
    s_config.vertices = vertices;
    int newShapeIndex = world.CreateShape(newActorIndex, s_config);
    cActor* newActor = world.p_actors[newActorIndex];
    shapes.push_back(newShapeIndex);
    actors.push_back(newActorIndex);
    return newActor;
}

cActor* CreateTriangleActor(float edgeLength, vec2& centrePos)
{
    float height = (sqrt(3.0f) / 2.0f) * edgeLength;

    // Equilateral triangle
	std::vector<vec2> vertices{
        vec2(0, (2.0f / 3.0f) * height),
        vec2(-edgeLength / 2.0f,  -(1.0f / 3.0f) * height),
        vec2(edgeLength / 2.0f, -(1.0f / 3.0f) * height)
	};
    ActorConfig a_config;
    a_config.position = centrePos;
    int newActorIndex = world.CreateActor(a_config);
    ShapeConfig s_config;
    s_config.vertices = vertices;
    int newShapeIndex = world.CreateShape(newActorIndex, s_config);
    cActor* newActor = world.p_actors[newActorIndex];
    shapes.push_back(newShapeIndex);
    actors.push_back(newActorIndex);
    return newActor;
}

cActor* CreatePolygonActor(std::vector<vec2>& inVertices, const vec2& centrePos)
{
    ActorConfig a_config;
    a_config.position = centrePos;
    int newActorIndex = world.CreateActor(a_config);
    ShapeConfig s_config;
    s_config.vertices = inVertices;
    int newShapeIndex = world.CreateShape(newActorIndex, s_config);
    cActor* newActor = world.p_actors[newActorIndex];
    shapes.push_back(newShapeIndex);
    actors.push_back(newActorIndex);
    return newActor;
}

void BoxScene()
{
    // Create a floor
    ActorConfig a_config;
    a_config.type = cActorType::STATIC;
    a_config.position = vec2{ 0, 3 };
    int floorActorIndex = world.CreateActor(a_config);
    ShapeConfig s_config;
    std::vector<vec2> vertices;
    vertices.push_back({ -5.0f, -0.25f });
    vertices.push_back({ 5.0f, -0.25f });
    vertices.push_back({ 5.0f, 0.25f });
    vertices.push_back({ -5.0f,  0.25f });
    s_config.vertices = vertices;
    int floorShapeIndex = world.CreateShape(floorActorIndex, s_config);
    shapes.push_back(floorShapeIndex);
    actors.push_back(floorActorIndex);

    a_config.type = cActorType::DYNAMIC;
    a_config.position = vec2{ 0, 2.25 };
    int boxAActorIndex = world.CreateActor(a_config);
    vertices.clear();
    vertices.push_back({ -0.5f, -0.5f });
    vertices.push_back({ 0.5f, -0.5f });
    vertices.push_back({ 0.5f, 0.5f });
    vertices.push_back({ -0.5f,  0.5f });
    s_config.vertices = vertices;
    int boxAShapeIndex = world.CreateShape(boxAActorIndex, s_config);
    shapes.push_back(boxAShapeIndex);
    actors.push_back(boxAActorIndex);
    
}

void InitPhysics()
{
    BoxScene();
 //   std::vector<vec2> verts{ {0, -0.577f}, {0.5f, 0.289f}, {-0.5f, 0.289f} };
	//cActor* poly = CreatePolygonActor(verts, vec2::zero);
 //   cTransform xfp = poly->getTransform();
 //   xfp.pos = { 0.0f, 2.0f };
 //   poly->setTransform(xfp);

 //   verts.clear();
 //   verts = { {-1,-1}, {1,-1}, {1,1}, {-1, 1} };
 //   cActor* box = CreatePolygonActor(verts, vec2::zero);
 
    // // create floor
	//CreateRectActor(recommendedWidth, 10, vec2{ recommendedWidth / 2.0f, recommendedHeight - 10.0f }, true);
}



void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    CP_System_ShowConsole();
	InitPhysics();
    
    DynamicTree t;
}

bool pauseStep = false;
bool isPaused = true;
void UpdatePhysics()
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
        
        if (CP_Input_KeyTriggered(KEY_SLASH))
            pauseStep = true;
        
        if (pauseStep)
        {
            world.step(1.0f / 50.0f);
            pauseStep = false;
        }
    }

    for (int itr = 0; itr < world.p_actors.size(); itr++)
    {
        cActor* a = world.p_actors[itr];
        cShape* s = world.p_shapes[a->GetShapeIndex()];
        DrawActor(s, a->getTransform());
    }

    for (int itr = 0; itr < world.p_contacts.size(); itr++)
    {
        cContact* c = world.p_contacts[itr];
        DrawContact(c);
    }

    
}

cActor* selectedActor;
void HandleInput(CP_Vector mousePosIn)
{
    CP_Vector mousePos = mousePosIn;// CP_Vector_Add(mousePosIn, { middle.x / 2, middle.y / 2 });
    if ((CP_Input_MouseDown(MOUSE_BUTTON_1) && !isHolding_mouse))
    {
        for (int itr = 0; itr < world.p_actors.size(); itr++)
        {
            cActor* a = world.p_actors[itr];
            cTransform tfm = a->getTransform();
            vec2 realCenter = (tfm.pos * 100) + middle;
            if (IsPointInRadius(CP_Vector{ realCenter.x, realCenter.y }, 50, mousePos))
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
                vec2 realCenter = (tfm.pos * 100) + middle;
                if (IsPointInRadius(CP_Vector{ realCenter.x, realCenter.y }, 50, mousePos))
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
        cTransform tfm = selectedActor->getTransform();
        tfm.pos = vec2{ mousePos.x - middle.x, mousePos.y - middle.y } / 100;
        selectedActor->setTransform(tfm);

        if (CP_Input_MouseDown(MOUSE_BUTTON_2))
        {
            cTransform tfm = selectedActor->getTransform();
            tfm.rot += (35 * DEG2RAD) * CP_System_GetDt();
            selectedActor->setTransform(tfm);
        }

        if (CP_Input_KeyTriggered(KEY_K))
        {
            selectedActor->addTorque(5000.0f);
        }
    }
    
    if (isHolding_keys && selectedActor)
    {
        cTransform tfm = selectedActor->getTransform();
        vec2& pos = tfm.pos;

        if (CP_Input_KeyDown(KEY_RIGHT))
        {
            tfm.pos += vec2::right * 0.1f;
        }
        else if (CP_Input_KeyDown(KEY_LEFT))
        {
            tfm.pos += vec2::left * 0.1f;
        }
        else if (CP_Input_KeyDown(KEY_UP))
        {
            tfm.pos += vec2::down * 0.1f;
        }
        else if (CP_Input_KeyDown(KEY_DOWN))
        {
            tfm.pos += vec2::up * 0.1f;
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
    UpdatePhysics();

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
