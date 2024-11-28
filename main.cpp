
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "vector2.h"
#include "cActor.h"
#include "cprocessing.h"
#include "physicsWorld.h"
#include "aabbtree.h"

using namespace chiori;
PhysicsWorld world; // create an instance of the physics world
bool isHolding = false;

CP_Color randomColors[] = {
    { 128, 0,   0,   255 },
    { 128, 128, 0,   255 },
    { 0,   128, 0,   255 },
    { 0,   128, 128, 255 },
    { 0,   0,   128, 255 },
    { 128, 0,   128, 255 } };

bool IsPointInRadius(CP_Vector center, float radius, CP_Vector pos)
{
    float distX = pos.x - center.x;
    float distY = pos.y - center.y;
    float distance = (distX * distX) + (distY * distY);
    return distance < (radius * radius);
}

void DrawActor(cActor*& inActor)
{
	const std::vector<vec2> vertices = inActor->getVertices();
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
    CP_Graphics_DrawCircle(inActor->position.x, inActor->position.y, 3);
}

void CreateRandomizedActor(int vertexCount, float radius, const vec2& centrePos) // TEMP!
{
    bool validAngle = false;
	std::vector<vec2> vertices;
    std::vector<float> angles;
    while (angles.size() < vertexCount)
    {
        validAngle = false;
        float newAngle = CP_Random_RangeFloat(0.0f, 2.0f * commons::PI);
        for (int i = 0; i < angles.size(); i++)
        {
            if ((newAngle <= angles[i] + 0.5f && newAngle >= angles[i] - 0.5f))
            {
                validAngle = true;
            }
        }
        if (!validAngle)
            angles.push_back(newAngle);
    }
    
    std::sort(angles.begin(), angles.end());

    for (int i = 0; i < vertexCount; i++)
    {
        float xcoord = radius * cos(angles[i]);
        float ycoord = radius * sin(angles[i]);
        vertices.push_back({ xcoord, ycoord });
    }

	cActor* newActor = world.AddActor(vertices);
    newActor->setMass(10);
    newActor->setPosition(centrePos);
}

void CreateRectActor(float width, float height, vec2& centrePos, bool isStatic = false)
{
    std::vector<vec2> vertices;
	vertices.push_back({ -width / 2.0f, -height / 2.0f });
	vertices.push_back({ width / 2.0f, -height / 2.0f });
	vertices.push_back({ width / 2.0f, height / 2.0f });
	vertices.push_back({ -width / 2.0f, height / 2.0f });

	cActor* newActor = world.AddActor(vertices);
    newActor->setMass(10);
    newActor->setPosition(centrePos);
    auto f = newActor->getFlags();
    f.toggle(cActor::USE_GRAVITY);
    newActor->setFlags(f);
	if (isStatic)
		newActor->setFlags(cActor::IS_STATIC);
}

void CreateTriangleActor(float radius, vec2& centrePos, bool isStatic = false)
{
    // Equilateral triangle
	std::vector<vec2> vertices{
		vec2 { 0, radius },
		vec2 { -radius, -radius },
		vec2 { radius, -radius }
	};
    cActor* newActor = world.AddActor(vertices);
    newActor->setMass(10);
    newActor->setPosition(centrePos);
    if (isStatic)
        newActor->setFlags(cActor::IS_STATIC);
}

//void CreateLineActor(float radius, vec2& centrePos, bool isStatic = false)
//{
//    std::vector<vec2> vertices{
//        vec2 { 0, radius },
//        vec2 { 0, -radius }
//    };
//    cActor& newActor = world.AddActor(vertices);
//    newActor.setMass(10);
//    newActor.setPosition(centrePos);
//    if (isStatic)
//        newActor.setFlags(cActor::IS_STATIC);
//}



const float EPSILON = 0.0000001f;
int recommendedWidth = 1600;
int recommendedHeight = 900;
bool drawColors = true;
bool drawFPS = true;

void InitPhysics()
{
    vec2 middle = vec2{ recommendedWidth / 2.0f, recommendedHeight / 2.0f };
  
    //CreateTriangleActor(50, middle);
    //CreateTriangleActor(50, vec2{ /*middle.x + 150, middle.y*/904, 510 });
    
    CreateRectActor(100, 100, middle);
	CreateRectActor(100, 100, vec2{ middle.x + 150, middle.y });
   // CreateRandomizedActor(5, 50, middle);
    //CreateRandomizedActor(5, 50, vec2{ middle.x, middle.y + 150 });
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

void UpdatePhysics()
{
    world.update(CP_System_GetDt());
    std::vector<cActor*>& actors = world.getWorldActors();
    for (cActor*& a : actors)
    {
        DrawActor(a);
        // keep within bounds
        if (a->position.x >= recommendedWidth) a->position.x = recommendedWidth;
        if (a->position.y >= recommendedHeight) a->position.y = recommendedHeight;
        if (a->position.x < 0) a->position.x = 0;
		if (a->position.y < 0) a->position.y = 0;
    }
}

cActor* selectedActor;
void HandleInput(CP_Vector mousePos)
{
    std::vector<cActor*>& actors = world.getWorldActors();
    if (CP_Input_MouseDown(MOUSE_BUTTON_1) && !isHolding)
    {
        for (cActor*& a : actors)
        {
            if (IsPointInRadius(CP_Vector{ a->getPosition().x, a->getPosition().y }, 50, mousePos))
            {
                selectedActor = a;
                isHolding = true;
                break;
            }
        }
    }

    if (isHolding && selectedActor)
    {
        selectedActor->setPosition(vec2{ mousePos.x, mousePos.y });
    }

    if (CP_Input_MouseDown(MOUSE_BUTTON_2) && isHolding && selectedActor)
    {
        selectedActor->setRotation(selectedActor->getRotation() + 35 * commons::DEG2RAD * CP_System_GetDt());
    }

    if (CP_Input_KeyTriggered(KEY_K) && isHolding && selectedActor)
    {
        selectedActor->addTorque(5000.0f);
    }
    
    if (CP_Input_MouseReleased(MOUSE_BUTTON_1))
    {
        selectedActor = nullptr;
        isHolding = false;
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
