
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
PhysicsWorld world; // create an instance of the physics world
bool isHolding_mouse = false;
bool isHolding_keys = false;

std::vector<cActor*> actors;
std::vector<cShape*> shapes;

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

void DrawActor(cShape*& inShape, const cTransform& inTfm)
{
	const std::vector<vec2> vertices = inShape->getVertices(inTfm);
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
    CP_Graphics_DrawCircle(inTfm.pos.x, inTfm.pos.y, 3);
}

void CreateRandomizedActor(int vertexCount, float radius, const vec2& centrePos) // TEMP!
{
    bool validAngle = false;
	std::vector<vec2> vertices;
    std::vector<float> angles;
    while (angles.size() < vertexCount)
    {
        validAngle = false;
        float newAngle = CP_Random_RangeFloat(0.0f, 2.0f * PI);
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
    cShape* newShape = world.CreateShape(vertices);
	cActor* newActor = world.CreateActor(newShape);
    newActor->setMass(10);
    cTransform tfm = newActor->getTransform();
    tfm.pos = centrePos;
    newActor->setTransform(tfm);
    shapes.push_back(newShape);
    actors.push_back(newActor);
}

cActor* CreateRectActor(float width, float height, vec2& centrePos, bool isStatic = false)
{
    std::vector<vec2> vertices;
	vertices.push_back({ -width / 2.0f, -height / 2.0f });
	vertices.push_back({ width / 2.0f, -height / 2.0f });
	vertices.push_back({ width / 2.0f, height / 2.0f });
	vertices.push_back({ -width / 2.0f, height / 2.0f });

    cShape* newShape = world.CreateShape(vertices);
    cActor* newActor = world.CreateActor(newShape);
    newActor->setMass(10);
    cTransform tfm = newActor->getTransform();
    tfm.pos = centrePos;
    newActor->setTransform(tfm);
    auto f = newActor->getFlags();
    f.toggle(cActor::USE_GRAVITY);
    newActor->setFlags(f);
    std::cout << "NEW SHAPE!" << std::endl;
    for (vec2& v : vertices)
    {
        std::cout << v + centrePos << std::endl;
    }
    shapes.push_back(newShape);
    actors.push_back(newActor);
    return newActor;
}

void CreateTriangleActor(float radius, vec2& centrePos, bool isStatic = false)
{
    // Equilateral triangle
	std::vector<vec2> vertices{
		vec2 { 0, radius },
		vec2 { -radius, -radius },
		vec2 { radius, -radius }
	};
    cShape* newShape = world.CreateShape(vertices);
    cActor* newActor = world.CreateActor(newShape);
    newActor->setMass(10);
    cTransform tfm = newActor->getTransform();
    tfm.pos = centrePos;
    newActor->setTransform(tfm);
    auto f = newActor->getFlags();
    f.toggle(cActor::USE_GRAVITY);
    newActor->setFlags(f);
    shapes.push_back(newShape);
    actors.push_back(newActor);
}


int recommendedWidth = 1600;
int recommendedHeight = 900;
bool drawColors = true;
bool drawFPS = true;

//void calculateCSO()
//{
//    std::cout << "CALCULATE CSO!" << std::endl;;
//    cTransform a = actors[0]->getTransform();
//    cTransform b = actors[1]->getTransform();
//    cShape* sa = shapes[0];
//    cShape* sb = shapes[1];
//
//    std::vector<vec2> a_verts = sa->getVertices(a);
//    std::vector<vec2> b_verts = sb->getVertices(b);
//    for (const vec2& va : a_verts)
//    {
//        for (const vec2& vb : b_verts)
//        {
//            Mvert w{ va, vb };
//            std::cout << w << std::endl;
//        }
//    }
//}


void InitPhysics()
{
    vec2 middle = vec2{ recommendedWidth / 2.0f, recommendedHeight / 2.0f };
  
    //CreateTriangleActor(50, middle);
    //CreateTriangleActor(50, vec2{ /*middle.x + 150, middle.y*/904, 510 });
    
    CreateRectActor(100, 100, middle);
	cActor* a = CreateRectActor(100, 100, vec2{ middle.x + 50, middle.y});
    cTransform xf = a->getTransform();
    xf.pos = vec2{ 726, 517 };
    xf.rot = 1.71739519;
    a->setTransform(xf);
    //CreateRectActor(100, 100, vec2{ middle.x, middle.y + 150 });

    //calculateCSO();
    //CreateTriangleActor(50, vec2{ middle.x - 150, middle.y });
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

    for (int itr = 0; itr < world.p_actors.size(); itr++)
    {
        cActor* a = world.p_actors[itr];
        cShape* s = world.p_shapes[a->GetShapeIndex()];
        DrawActor(s, a->getTransform());
        cTransform tfm = a->getTransform();
        vec2& pos = tfm.pos;
        if (pos.x >= recommendedWidth) pos.x = recommendedWidth;
        if (pos.y >= recommendedHeight) pos.y = recommendedHeight;
        if (pos.x < 0) pos.x = 0;
        if (pos.y < 0) pos.y = 0;
        a->setTransform(tfm);
    }
}

cActor* selectedActor;
void HandleInput(CP_Vector mousePos)
{
    if ((CP_Input_MouseDown(MOUSE_BUTTON_1) && !isHolding_mouse))
    {
        for (int itr = 0; itr < world.p_actors.size(); itr++)
        {
            cActor* a = world.p_actors[itr];
            cTransform tfm = a->getTransform();
            if (IsPointInRadius(CP_Vector{ tfm.pos.x, tfm.pos.y }, 50, mousePos))
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
                if (IsPointInRadius(CP_Vector{ tfm.pos.x, tfm.pos.y }, 50, mousePos))
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
        tfm.pos = vec2{ mousePos.x, mousePos.y };
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
            tfm.pos += vec2::right * 0.01f;
        }
        else if (CP_Input_KeyDown(KEY_LEFT))
        {
            tfm.pos += vec2::left * 0.01f;
        }
        else if (CP_Input_KeyDown(KEY_UP))
        {
            tfm.pos += vec2::up * 0.01f;
        }
        else if (CP_Input_KeyDown(KEY_DOWN))
        {
            tfm.pos += vec2::down * 0.01f;
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
