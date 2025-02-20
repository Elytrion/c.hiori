#include "pch.h"
#include "voronoiscenemanager.h"
#include "cprocessing.h"
#include "graphics.h"
using namespace chiori;


void VoronoiScene::Update(float dt)
{
    if (drawTriangles)
    { // delaunay drawing
        CP_Settings_StrokeWeight(1);
        CP_Settings_Stroke(CP_Color_Create(255, 0, 0, 255)); // Red for Delaunay edges

        for (const auto& tri : tris) {
            cVec2 a = tri[0];
            cVec2 b = tri[1];
            cVec2 c = tri[2];
            CP_Graphics_DrawLine(a.x, a.y, b.x, b.y);
            CP_Graphics_DrawLine(b.x, b.y, c.x, c.y);
            CP_Graphics_DrawLine(c.x, c.y, a.x, a.y);
        }
    }

    if (drawVoronoi)
    {
        CP_Color cellColor = CP_Color_Create(255, 255, 255, 255);
        for (const auto& vvertex : voronoi.vertices)
        {
            for (const auto& edgeIndex : vvertex.edgeIndices)
            {
                const cVEdge& edge = voronoi.edges[edgeIndex];
                if (edge.infinite)
                {
                    const cVec2& p1 = edge.origin;
                    const cVec2& p2 = edge.origin + edge.endDir * 1000;
                    CP_Settings_StrokeWeight(1);
                    CP_Settings_Stroke(CP_Color_Create(255, 0, 255, 255)); // Green for Delaunay edges
                    CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                    continue;
                }
                const cVec2& p1 = edge.origin;
                const cVec2& p2 = edge.endDir;
                CP_Settings_StrokeWeight(1);
                CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255)); // Green for Delaunay edges
                CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
            }
        }
    }
    
    if (drawPoints)
    {
        for (const auto& p : voronoi.v_points)
        {
            CP_Settings_Fill(CP_Color_Create(255, 255, 0, 255));
            CP_Settings_Stroke(CP_Color_Create(255, 255, 0, 255));
            CP_Graphics_DrawCircle(p.x, p.y, 5);
        }
    }
}

void VoronoiScene::Unload()
{
    voronoi.clear();
    tris.clear();
}

#pragma region Scenes

class StaticScene : public VoronoiScene
{
    std::vector<cVec2> points;
    int numPoints = 100;
public:
    StaticScene(DebugGraphics* drawer) : VoronoiScene(drawer) {}

    void Load() override
    {
        int bufferEdgeWidth = 80;
        for (int i = 0; i < numPoints; i++) {
            float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
            float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth);
            points.push_back({ x,y });
        }
        tris = cVoronoiDiagram::triangulateDelaunator(points);
        voronoi.create(points.data(), points.size());
    }

    void Unload() override
    {
        VoronoiScene::Unload();
        points.clear();
    }
};

class GrowingScene : public VoronoiScene
{
    std::vector<cVec2> points;
    std::vector<cVec2> u_points;
    int numPoints = 100;
    int initPoints = 5;
    float baseTimer = 0.05f;
    float timer = 0.0f;
public:
    GrowingScene(DebugGraphics* drawer) : VoronoiScene(drawer) {}
    
    void Load() override
    {
        int bufferEdgeWidth = 80;
        for (int i = 0; i < numPoints; i++) {
            float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
            float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth);
            if (i < initPoints)
                u_points.push_back({ x, y });
            else
                points.push_back({ x, y });
        }
        tris = cVoronoiDiagram::triangulateDelaunator(u_points);
        voronoi.create(u_points.data(), u_points.size());
    }

    void Update(float dt) override
    {
        if (u_points.size() < numPoints)
        {
            timer -= dt;
            if (timer <= 0.0f)
            {
                timer = baseTimer;
                // move the last point of points to used_points
                // then retriangulate
                u_points.push_back(std::move(points.front()));
                points.erase(points.begin());
                tris.clear();
                voronoi.clear();
                tris = cVoronoiDiagram::triangulateDelaunator(u_points);
                voronoi.create(u_points.data(), u_points.size());
            }
        }

        if (drawTriangles)
        { // delaunay drawing
            CP_Settings_StrokeWeight(1);
            CP_Settings_Stroke(CP_Color_Create(255, 0, 0, 255)); // Red for Delaunay edges

            for (const auto& tri : tris) {
                cVec2 a = tri[0];
                cVec2 b = tri[1];
                cVec2 c = tri[2];
                CP_Graphics_DrawLine(a.x, a.y, b.x, b.y);
                CP_Graphics_DrawLine(b.x, b.y, c.x, c.y);
                CP_Graphics_DrawLine(c.x, c.y, a.x, a.y);
            }
        }

        if (drawVoronoi)
        {
            CP_Color cellColor = CP_Color_Create(255, 255, 255, 255);
            for (const auto& vvertex : voronoi.vertices)
            {
                for (const auto& edgeIndex : vvertex.edgeIndices)
                {
                    const cVEdge& edge = voronoi.edges[edgeIndex];
                    if (edge.infinite)
                    {
                        const cVec2& p1 = edge.origin;
                        const cVec2& p2 = edge.origin + edge.endDir * 1000;
                        CP_Settings_StrokeWeight(1);
                        CP_Settings_Stroke(CP_Color_Create(255, 0, 255, 255)); // Green for Delaunay edges
                        CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                        continue;
                    }
                    const cVec2& p1 = edge.origin;
                    const cVec2& p2 = edge.endDir;
                    CP_Settings_StrokeWeight(1);
                    CP_Settings_Stroke(CP_Color_Create(0, 255, 0, 255)); // Green for Delaunay edges
                    CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                }
            }
        }

        if (drawPoints)
        {
            for (const auto& p : points)
            {
                CP_Settings_Fill(CP_Color_Create(0, 255, 255, 255));
                CP_Settings_Stroke(CP_Color_Create(0, 255, 255, 255));
                CP_Graphics_DrawCircle(p.x, p.y, 10);
            }
            
            for (const auto& up : u_points)
            {
                CP_Settings_Fill(CP_Color_Create(255, 255, 0, 255));
                CP_Settings_Stroke(CP_Color_Create(255, 255, 0, 255));
                CP_Graphics_DrawCircle(up.x, up.y, 5);
            }
        }
    }

    void Unload() override
    {
        VoronoiScene::Unload();
        points.clear();
        u_points.clear();
        timer = 0.0f;
    }
};

class ModifiableScene : public VoronoiScene
{
    std::vector<cVec2> points;
    int numPoints = 100;
    cVec2* selectedPoint{ nullptr };
    cVec2 originalPoint{ cVec2::zero };
    float baseTimer = 0.1f;
    float timer = 0.0f;
public:
    ModifiableScene(DebugGraphics* drawer) : VoronoiScene(drawer) {}

    void Load() override
    {
        int bufferEdgeWidth = 80;
        for (int i = 0; i < numPoints; i++) {
            float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
            float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth);
            points.push_back({ x, y });
        }
        tris = cVoronoiDiagram::triangulateDelaunator(points);
        voronoi.create(points.data(), points.size());
    }

    void draw()
    {
        if (drawTriangles)
        { // delaunay drawing
            CP_Settings_StrokeWeight(1);
            CP_Settings_Stroke(CP_Color_Create(255, 0, 0, 255)); // Red for Delaunay edges

            for (const auto& tri : tris) {
                cVec2 a = tri[0];
                cVec2 b = tri[1];
                cVec2 c = tri[2];
                CP_Graphics_DrawLine(a.x, a.y, b.x, b.y);
                CP_Graphics_DrawLine(b.x, b.y, c.x, c.y);
                CP_Graphics_DrawLine(c.x, c.y, a.x, a.y);
            }
        }

        if (drawVoronoi)
        {
            for (const auto& vvertex : voronoi.vertices)
            {
                for (const auto& edgeIndex : vvertex.edgeIndices)
                {
                    const cVEdge& edge = voronoi.edges[edgeIndex];
                    if (edge.infinite)
                    {
                        const cVec2& p1 = edge.origin;
                        const cVec2& p2 = edge.origin + edge.endDir * 1000;
                        CP_Settings_StrokeWeight(1);
                        CP_Settings_Stroke(CP_Color_Create(0, 255, 255, 255)); // Green for Delaunay edges
                        CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                        continue;
                    }
                    const cVec2& p1 = edge.origin;
                    const cVec2& p2 = edge.endDir;
                    CP_Settings_StrokeWeight(1);
                    CP_Settings_Stroke(CP_Color_Create(255, 0, 255, 255)); // Green for Delaunay edges
                    CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                }
            }
        }

        if (drawPoints)
        {
            for (const auto& p : points)
            {
                CP_Settings_Fill(CP_Color_Create(255, 255, 0, 255));
                CP_Graphics_DrawCircle(p.x, p.y, 10);
            }
        }
    }

    void Update(float dt) override
    {
        cVec2 mousePos = { CP_Input_GetMouseX(), CP_Input_GetMouseY() };
        draw();
        if (CP_Input_MouseDown(MOUSE_BUTTON_2))
        {
            if (!selectedPoint)
            {
                for (cVec2& pt : points)
                {
                    if (distanceSqr(pt, mousePos) < 25.0f) // 5 ^ 2
                    {
                        selectedPoint = &pt;
                        originalPoint = pt;
                        break;
                    }
                }
                return;
            }
            
            selectedPoint->x = mousePos.x;
            selectedPoint->y = mousePos.y;

            if (distanceSqr(*selectedPoint, originalPoint) <= 4.0f)
                return; // hasn't move far enough to care about changing;
            
            timer -= dt;
            if (timer <= 0.0f)
            {
                timer = baseTimer;
                voronoi.remove(originalPoint, false);
                voronoi.add(*selectedPoint);
                originalPoint = *selectedPoint;
                tris = cVoronoiDiagram::triangulateDelaunator(voronoi.v_points);
            }
        }
        else if (CP_Input_MouseReleased(MOUSE_BUTTON_2))
        {
            if (!selectedPoint)
                return;
            
            voronoi.remove(originalPoint, false);
            voronoi.add(*selectedPoint);
            originalPoint = cVec2::zero;
            selectedPoint = nullptr;
            tris = cVoronoiDiagram::triangulateDelaunator(voronoi.v_points);
            return;
        }

        if (CP_Input_KeyTriggered(KEY_N))
        {
            points.push_back(mousePos);
            voronoi.add(mousePos);
        }
        else if (CP_Input_KeyTriggered(KEY_M))
        {
            auto itr = std::find_if(points.begin(), points.end(), [&](const cVec2& p) {
                return distanceSqr(p, mousePos) <= 4.0f;
                });
            if (itr == points.end())
                return; // point not in diagram, ignore
            cVec2 fpoint = *itr;
            points.erase(itr);
            voronoi.remove(fpoint);
        }

        if (CP_Input_KeyTriggered(KEY_S))
        {
            cVoronoiDiagram::save("saved_scene", voronoi);
        }
        else if (CP_Input_KeyTriggered(KEY_L))
        {
            voronoi.clear();
            points.clear();
            voronoi = cVoronoiDiagram::load("saved_scene");
            points = voronoi.v_points;
            tris = cVoronoiDiagram::triangulateDelaunator(voronoi.v_points);
        }
    }
    
    void Unload() override
    {
        VoronoiScene::Unload();
        points.clear();
        selectedPoint = nullptr;
        originalPoint = cVec2::zero;
    }
};
#pragma endregion

VoronoiSceneManager::VoronoiSceneManager(DebugGraphics* drawer)
    : drawer(drawer)
{

    AddScene(new StaticScene(drawer));
    AddScene(new GrowingScene(drawer));
    AddScene(new ModifiableScene(drawer));

    //ChangeScene(0);
}

VoronoiSceneManager::~VoronoiSceneManager()
{
    for (int i = scenes.size() - 1; i >= 0; i--)
    {
        delete scenes[i];
    }
}

void VoronoiSceneManager::ChangeScene(int sceneIndex)
{
    if (currentScene >= 0)
    {
        scenes[currentScene]->Unload();
    }
    currentScene = sceneIndex;
    scenes[currentScene]->Load();
}

void VoronoiSceneManager::AddScene(VoronoiScene* scene)
{
    sceneCount++;
    scenes.push_back(scene);
}

void VoronoiSceneManager::RemoveScene(int sceneIndex)
{
    if (sceneIndex >= 0 && sceneIndex < sceneCount)
    {
        delete scenes[sceneIndex];
        scenes.erase(scenes.begin() + sceneIndex);
        sceneCount--;
    }
}

VoronoiScene* VoronoiSceneManager::GetCurrentScene()
{
    return scenes[currentScene];
}

void VoronoiSceneManager::Update(float dt)
{
    if (CP_Input_KeyTriggered(KEY_ESCAPE))
    {
        ChangeScene(currentScene);
        return;
    }

    VoronoiScene* scene = GetCurrentScene();
    if (CP_Input_KeyTriggered(KEY_T))
    {
        scene->drawTriangles = !scene->drawTriangles;
    }
    if (CP_Input_KeyTriggered(KEY_V))
    {
        scene->drawVoronoi = !scene->drawVoronoi;
    }
    if (CP_Input_KeyTriggered(KEY_P))
    {
        scene->drawPoints = !scene->drawPoints;
    }
    scene->Update(dt);
}