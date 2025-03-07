#include "pch.h"
#include "voronoiscenemanager.h"
#include "cprocessing.h"
#include "graphics.h"
#include "aabb.h"
#include "fractureWorld.h"
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
                    if (drawInfinite)
                    {
                        const cVec2& p1 = edge.origin;
                        const cVec2& p2 = edge.origin + edge.endDir * 1000;
                        CP_Settings_StrokeWeight(1);
                        CP_Settings_Stroke(CP_Color_Create(255, 0, 255, 255)); // Green for Delaunay edges
                        CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
                    }
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

class MovingScene : public VoronoiScene
{
    std::vector<cVec2> seeds; // not the points the diagram uses
    std::vector<cVec2> points; // these are the actual points that will orbit the seeds
    int numPoints = 55;

    std::vector<float> times;
    std::vector<cVec2> ellipses;
    
    float baseTimer = 0.025f;
    float timer = 0.0f;
    float speed = 1.0f;

public:
    MovingScene(DebugGraphics* drawer) : VoronoiScene(drawer) {}

    void Load() override
    {
        drawVoronoi = false;
        drawTriangles = false;
        drawInfinite = false;
        int bufferEdgeWidth = 200;
        for (int i = 0; i < numPoints; i++) {
            float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
            float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth);
            seeds.push_back({ x, y });
            points.push_back({ x, y });

            times.push_back(CP_Random_RangeFloat(0.0f, 2.0f * PI)); // random angle
            int circle = CP_Random_RangeInt(0, 5);
            float sm_a = CP_Random_RangeFloat(5.0f, 35.0f);
            float sm_b = CP_Random_RangeFloat(5.0f, 35.0f);
            if (circle < 2)
            {
                ellipses.push_back({ sm_a, sm_a });
            }
            else
            {
                ellipses.push_back({ sm_a, sm_b });
            }
        }
    }
    // Function to update a point's position along an elliptical path
    cVec2 updateEllipticalPath(float t, float a, float b, float centerX, float centerY) {
        return { centerX + a * cos(t), centerY + b * sin(t) };
    }

    void Update(float dt) override
    {
        if (CP_Input_KeyTriggered(KEY_N))
        {
            speed += 0.5f;
        }
        if (CP_Input_KeyTriggered(KEY_M))
        {
            speed -= 0.5f;
        }

        for (int i = 0; i < numPoints; ++i) {
            float& time = times[i];
			const cVec2& ellipse = ellipses[i];
            time += dt;
            if (time > 2 * PI)
            {
                time -= 2 * PI;
            }
            points[i] = updateEllipticalPath(time * speed, ellipse.x, ellipse.y, seeds[i].x, seeds[i].y);
        }

        VoronoiScene::Update(dt);

        timer -= dt;
        if (timer > 0.0f)
            return;
        
        timer = baseTimer;
        tris.clear();
        voronoi.clear();
        tris = cVoronoiDiagram::triangulateDelaunator(points);
        voronoi.create(points.data(), points.size());
    }

    void Unload() override
    {
        VoronoiScene::Unload();
        seeds.clear();
        points.clear();
        times.clear();
        ellipses.clear();
        timer = baseTimer;
        speed = 1.0f;
    }
};

class CutScene : public VoronoiScene
{
    std::vector<cVec2> points;
    int numPoints = 6;
    float extents = 80.0f;
    cAABB aabb;
    bool hasCut = false;

    std::vector<cVCell> cells;

    std::vector<cVec2> clippedVerts{};

    bool drawCellsAll = false;
    bool drawCellsSingle = true;
    
    int currentCell = 0;

public:
    CutScene(DebugGraphics* drawer) : VoronoiScene(drawer) {}

    void Load() override
    {
        int bufferEdgeWidth = 400;
        for (int i = 0; i < numPoints; i++) {
            float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
            float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth / 2);
            points.push_back({ x,y });
        }
        //points = { {825, 481},
        //{900, 498},
        //{ 908, 478 },
        //{ 468, 615 },
        //{ 534, 602 },
        //{ 664, 545 } };
        
        //points = { {909, 598},
        //        {673, 418},
        //        {916, 675},
        //        {412, 529},
        //        {459, 509},
        //        {886, 670} };

        //points = { {1111, 408},
        //            {928, 683},
        //            {481, 451},
        //            {808, 594},
        //            {1132, 435},
        //            {1141, 554} };
        
        tris = cVoronoiDiagram::triangulateDelaunator(points);
        voronoi.create(points.data(), points.size());

        aabb = CreateAABB(extents, extents);
        hasCut = false;


        cells = voronoi.getCells();
    }

    void Update(float dt) override
    {
        // update the position of the AABB
        cVec2 mousePos { CP_Input_GetMouseX(), CP_Input_GetMouseY() };
        const cVec2 extent = cVec2{ extents, extents };
		aabb.min = mousePos - extent;
        aabb.max = mousePos + extent;
        cAABB exaabb = aabb;
        exaabb.min = aabb.min - (aabb.getExtents());
        exaabb.max = aabb.max + (aabb.getExtents());
        cVec2 center = aabb.getCenter();
        CP_Settings_StrokeWeight(2);
        CP_Settings_Stroke(CP_Color_Create(255, 0, 255, 255));
        CP_Graphics_DrawLine(aabb.min.x, aabb.min.y, aabb.min.x, aabb.max.y);
        CP_Graphics_DrawLine(aabb.min.x, aabb.max.y, aabb.max.x, aabb.max.y);
        CP_Graphics_DrawLine(aabb.max.x, aabb.max.y, aabb.max.x, aabb.min.y);
        CP_Graphics_DrawLine(aabb.max.x, aabb.min.y, aabb.min.x, aabb.min.y);
        CP_Graphics_DrawCircle(center.x, center.y, 3);
        CP_Settings_Stroke(CP_Color_Create(200, 100, 255, 255));
        CP_Graphics_DrawLine(exaabb.min.x, exaabb.min.y, exaabb.min.x, exaabb.max.y);
        CP_Graphics_DrawLine(exaabb.min.x, exaabb.max.y, exaabb.max.x, exaabb.max.y);
        CP_Graphics_DrawLine(exaabb.max.x, exaabb.max.y, exaabb.max.x, exaabb.min.y);
        CP_Graphics_DrawLine(exaabb.max.x, exaabb.min.y, exaabb.min.x, exaabb.min.y);

       
        //if (cells.size() > 0)
        //{
        //    auto drawCell = [&](const cVCell& cell)
        //        {
        //            CP_Settings_Fill(CP_Color_Create(50, 50, 200, 255)); // Normal cells in blue

        //            if (cell.infinite)
        //            {
        //                CP_Settings_Fill((cell.vertices.size() != 1) ? CP_Color_Create(200, 50, 50, 128)  // Infinite cells in red
        //                    : CP_Color_Create(50, 200, 100, 255)); // degen cells in blue
        //            }
        //            CP_Graphics_BeginShape();
        //            
        //            std::vector<cVec2> verts;
        //            for (size_t i = 0; i < cell.vertices.size(); i++)
        //            {
        //                const cVVert& v = cell.vertices[i];

        //                if (cell.infinite)
        //                {
        //                    if (cell.vertices.size() == 1) // degen cell
        //                    {
        //                        // vert is the order of 
        //                        // inf edge 1 , self, inf edge 2
        //                        // do we know order who the fuck knows we just pray first its first to last
        //                        cVec2 extendedA = v.site, extendedB = v.site;
        //                        bool hasA = false;
        //                        for (unsigned i : v.edgeIndices)
        //                        {
        //                            if (voronoi.edges[i].infinite)
        //                            {
        //                                if (hasA)
        //                                    extendedB += voronoi.edges[i].endDir * 1000.0f;
        //                                else
        //                                {
        //                                    extendedA += voronoi.edges[i].endDir * 1000.0f;
        //                                    hasA = true;
        //                                }
        //                            }
        //                        }
        //                        //verts.push_back(extendedA);
        //                        //verts.push_back(v.site);
        //                        //verts.push_back(extendedB);
        //                        
        //                    }
        //                    else
        //                    {
        //                        // we are guaranteed the first and second are the infinite verts
        //                        if (i == 0 || i == 1)
        //                        {
        //                            cVec2 extended = v.site;     
        //                            float closer = -FLT_MAX;
        //                            for (unsigned i : v.edgeIndices)
        //                            {
        //                                if (voronoi.edges[i].infinite)
        //                                {
        //                                    // Compare direction of edge with vector to seed
        //                                    cVec2 edgeDir = voronoi.edges[i].endDir.normalized();
        //                                    cVec2 toSeed = (voronoi.v_points[cell.seedIndex] - v.site).normalized();
        //                                    float dot = edgeDir.dot(toSeed);

        //                                    // Higher dot means better alignment
        //                                    if (dot > closer)
        //                                    {
        //                                        closer = dot;
        //                                        extended = voronoi.edges[i].origin + voronoi.edges[i].endDir * 1000.0f;
        //                                    }
        //                                }
        //                            }
        //                            if (i == 0)
        //                            {
        //                                verts.push_back(v.site);
        //                                verts.push_back(extended);
        //                            }
        //                            else
        //                            {
        //                                verts.push_back(extended);
        //                                verts.push_back(v.site);
        //                            }
        //                        }
        //                        else
        //                            verts.push_back(v.site);
        //                    }
        //                }
        //                else
        //                    verts.push_back(v.site);
        //            }

        //            
        //            for (auto& pt : verts)
        //            {
        //                CP_Graphics_AddVertex(pt.x, pt.y);
        //            }
        //            CP_Graphics_EndShape();
        //            

        //        };
        //}
   
        auto drawCell = [&](const cVCell& cell)
            {
                std::vector<cVec2> verts;
                
                for (size_t i = 0; i < cell.vertices.size(); i++)
                {
                    const cVVert& v = voronoi.vertices[cell.vertices[i]];

                    if (!cell.infinite)
                    {
                        verts.push_back(v.site);
                        continue;
                    }

                    if (cell.vertices.size() == 1) // degen cell
                    {
                        // vert is the order of 
                        // inf edge 1 , self, inf edge 2
                        // do we know order who the fuck knows we just pray first its first to last
                        cVec2 extendedA = v.site, extendedB = v.site;
                        bool hasA = false;
                        for (unsigned i : v.edgeIndices)
                        {
                            if (voronoi.edges[i].infinite)
                            {
                                if (hasA)
                                    extendedB += voronoi.edges[i].endDir * 1000.0f;
                                else
                                {
                                    extendedA += voronoi.edges[i].endDir * 1000.0f;
                                    hasA = true;
                                }
                            }
                        }
                        verts.push_back(extendedA);
                        verts.push_back(v.site);
                        verts.push_back(extendedB);
                        continue;
                    }

                    const cVVert& infVA = voronoi.vertices[cell.vertices[cell.infVertA]];
                    const cVVert& infVB = voronoi.vertices[cell.vertices[cell.infVertB]];
                    const cVEdge& infEA = voronoi.edges[cell.infEdgeA];
                    const cVEdge& infEB = voronoi.edges[cell.infEdgeB];

                    const cVec2 exA = infVA.site + (infEA.endDir.normalized() * 10000.0f);
                    const cVec2 exB = infVB.site + (infEB.endDir.normalized() * 10000.0f);

                    verts.push_back(exA);          // Extended infinite edge A
                    verts.push_back(infVA.site);    // Infinite vertex A
                    verts.push_back(infVB.site);    // Infinite vertex B
                    verts.push_back(exB);          // Extended infinite edge B

                    for (size_t j = 0; j < cell.vertices.size(); j++)
                    {
                        if (j != cell.infVertA && j != cell.infVertB)
                        {
                            verts.push_back(voronoi.vertices[cell.vertices[j]].site);
                        }
                    }
                    const cVec2& seedPt = voronoi.v_points[cell.seedIndex];
                    std::sort(verts.begin(), verts.end(), [seedPt](const cVec2& a, const cVec2& b)
                        {
                            return atan2(a.y - seedPt.y, a.x - seedPt.x) < atan2(b.y - seedPt.y, b.x - seedPt.x);
                        });
                    
                    //// A in magenta
                    //CP_Settings_Fill(CP_Color_Create(255, 50, 255, 255));
                    //CP_Settings_Stroke(CP_Color_Create(255, 50, 255, 255));
                    //CP_Graphics_DrawCircle(infVA.site.x, infVA.site.y, 5);
                    //// B in cyan
                    //CP_Settings_Fill(CP_Color_Create(50, 255, 255, 255));
                    //CP_Settings_Stroke(CP_Color_Create(50, 255, 255, 255));
                    //CP_Graphics_DrawCircle(infVB.site.x, infVB.site.y, 5);
                    //
                    //CP_Settings_StrokeWeight(3);
                    //
                    //CP_Settings_Stroke(CP_Color_Create(255, 50, 255, 255)); // A in magenta
                    //CP_Graphics_DrawLine(infVA.site.x, infVA.site.y, exA.x, exA.y);
                    //CP_Settings_Stroke(CP_Color_Create(50, 255, 255, 255)); // B in cyan
                    //CP_Graphics_DrawLine(infVB.site.x, infVB.site.y, exB.x, exB.y);
                }

                CP_Settings_Fill(CP_Color_Create(50, 50, 200, 255)); // Normal cells in blue
                if (cell.infinite)
                {
                    CP_Settings_Fill((cell.vertices.size() != 1) ? CP_Color_Create(200, 50, 50, 128)  // Infinite cells in red
                        : CP_Color_Create(50, 200, 100, 255)); // degen cells in whatever
                }            
                CP_Graphics_BeginShape();
                for (auto& pt : verts)
                {
                    CP_Graphics_AddVertex(pt.x, pt.y);
                }
                CP_Graphics_EndShape();

            };

        if (cells.size() > 0)
        {
            if (drawCellsSingle)
            {
                auto& cell = cells[currentCell];
                drawCell(cell);
            }
            else if (drawCellsAll)
            {
                for (auto& cell : cells)
                {
                    drawCell(cell);
                }
            }
        }
        
        if (CP_Input_KeyTriggered(KEY_S))
        {
            drawCellsAll = !drawCellsAll;
            drawCellsSingle = !drawCellsSingle;
        }
        if (drawCellsSingle)
        {
            auto& pt = voronoi.v_points[cells[currentCell].seedIndex];
            CP_Settings_Fill(CP_Color_Create(255, 255, 255, 255));
            CP_Settings_Stroke(CP_Color_Create(255, 255, 255, 255));
            CP_Graphics_DrawCircle(pt.x, pt.y, 5);
            cVec2 trueCentriod;
            for (auto& vi : cells[currentCell].vertices)
            {
                trueCentriod += voronoi.vertices[vi].site;
            }
            trueCentriod /= cells[currentCell].vertices.size();
            CP_Settings_Stroke(CP_Color_Create(0, 0, 0, 255));
            CP_Graphics_DrawCircle(trueCentriod.x, trueCentriod.y, 5);
        }
        if (CP_Input_KeyTriggered(KEY_I))
        {
            currentCell = (currentCell + 1) % cells.size();
            std::cout << "\nSHOWING CELL " << currentCell << " with SEED " 
                << cells[currentCell].seedIndex << "(" << voronoi.v_points[cells[currentCell].seedIndex] 
                << ")" << std::endl;
            if (cells[currentCell].infinite)
            {
                auto& cell = cells[currentCell];
                std::cout << "Inf Verts: " << cell.infVertA << ", " << cell.infVertB << " | " << cell.vertices.size() << std::endl;
                std::cout << "Inf Edges: " << cell.infEdgeA << ", " << cell.infEdgeB << " | " << voronoi.edges.size() << std::endl;
            }
        }
        if (CP_Input_KeyTriggered(KEY_Z))
        {
            for (auto& pt : voronoi.v_points)
            {
                std::cout << pt << std::endl;
            }
        }

        if (CP_Input_KeyTriggered(KEY_C) && !hasCut)
        {
            hasCut = true;
            cFracturePattern fp;
            cFractureWorld::CreateFracturePattern(fp, voronoi, aabb, false);
            tris.clear();
            voronoi.clear();
            voronoi = fp.pattern;
            cPolygon poly = GeomMakeBox(aabb.min, aabb.max);
            clippedVerts = ClipVoronoiWithPolygon(voronoi, poly.vertices, poly.normals, poly.count);
            tris = cVoronoiDiagram::triangulateDelaunator(voronoi.v_points);
            cells = voronoi.getCells();
        }

        VoronoiScene::Update(dt);

        if (clippedVerts.size() > 0)
        {
            for (auto& v : clippedVerts)
            {
                CP_Settings_Stroke(CP_Color_Create(0, 0, 255, 255));
                CP_Settings_Fill(CP_Color_Create(0, 0, 255, 255));
                CP_Graphics_DrawCircle(v.x, v.y, 5);
            }
        }
    }

    void Unload() override
    {
        VoronoiScene::Unload();
        points.clear();
        clippedVerts.clear();
        cells.clear();
        hasCut = false;
    }
};
#pragma endregion

VoronoiSceneManager::VoronoiSceneManager(DebugGraphics* drawer)
    : drawer(drawer)
{
    AddScene(new CutScene(drawer));
    AddScene(new StaticScene(drawer));
    AddScene(new GrowingScene(drawer));
    AddScene(new ModifiableScene(drawer));
    AddScene(new MovingScene(drawer));

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
    cVec2 displayDim = { 1600, 900 };
    char buffer[64];
    CP_Color textColor = CP_Color{ 50, 255, 50, 255 };
    float x = displayDim.x - 250;
    float y = 20;
    snprintf(buffer, 64, "Press T to toggle Triangulation Overlay");
    CP_Settings_Fill(textColor);
    CP_Settings_TextSize(15);
    CP_Font_DrawText(buffer, x, y);
    y += 20;

    snprintf(buffer, 64, "Press V to toggle Voronoi Overlay");
    CP_Settings_Fill(textColor);
    CP_Settings_TextSize(15);
    CP_Font_DrawText(buffer, x, y);
    y += 20;

    snprintf(buffer, 64, "Press P to toggle Point overlay");
    CP_Settings_Fill(textColor);
    CP_Settings_TextSize(15);
    CP_Font_DrawText(buffer, x, y);
    y += 20;
}