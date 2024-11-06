
#include <cstdio>
#include <cstdbool>
#include "pch.h"
#include "vector2.h"
#include "cActor.h"
#include "cprocessing.h"
#include "physicsWorld.h"

using namespace chiori;
PhysicsWorld world; // create an instance of the physics world

CP_Color randomColors[] = {
    { 128, 0,   0,   255 },
    { 128, 128, 0,   255 },
    { 0,   128, 0,   255 },
    { 0,   128, 128, 255 },
    { 0,   0,   128, 255 },
    { 128, 0,   128, 255 } };


void DrawActor(const cActor& inActor)
{
	const std::vector<vec2>& baseVertices = inActor.baseVertices;
    for (int i = 0; i < baseVertices.size(); i++)
    {
        CP_Settings_Fill(CP_Color_Create(255, 127, 127, 255));
        vec2 vert = inActor.position + baseVertices[i];
        vec2 nxtVert = inActor.position + baseVertices[(i + 1) % baseVertices.size()];
        CP_Graphics_DrawCircle(inActor.position.x, inActor.position.y, 10);
        CP_Graphics_DrawCircle(vert.x, vert.y, 5);
        CP_Settings_Stroke(CP_Color_Create(255, 127, 127, 255));
        CP_Graphics_DrawLine(vert.x, vert.y, nxtVert.x, nxtVert.y);
    }
}

void CreateActor(int vertexCount, float radius, const vec2& centrePos) // TEMP!
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

	cActor& newActor = world.AddActor();
    newActor.baseVertices = vertices;
    newActor.position = centrePos;
}



const float EPSILON = 0.0000001f;
int recommendedWidth = 1600;
int recommendedHeight = 900;
bool drawColors = true;
bool drawFPS = true;

void InitPhysics()
{
    vec2 middle = vec2{ recommendedWidth / 2.0f, recommendedHeight / 2.0f };
    CreateActor(10,50, middle);
}



void game_init(void)
{
    CP_System_SetWindowSize(recommendedWidth, recommendedHeight);
    CP_System_SetFrameRate(60.0f);
    //CP_System_ShowConsole();
	InitPhysics();
}

void UpdatePhysics()
{
    world.update(CP_System_GetDt());
    const std::vector<cActor>& actors = world.getWorldActors();
    for (const cActor& a : actors)
    {
        DrawActor(a);
    }
}

void game_update(void)
{
    CP_Settings_BlendMode(CP_BLEND_ALPHA);
    CP_Graphics_ClearBackground(CP_Color_Create(51, 51, 51, 255));
    CP_Settings_Fill(CP_Color_Create(0, 0, 0, 255));

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

int main(void)
{
    CP_Engine_SetNextGameState(game_init, game_update, game_exit);
    CP_Engine_Run();
    return 0;
}
