#pragma once
#include <windows.h>
#include <commdlg.h> // Windows file dialog
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include "fractureWorld.h"

using namespace chiori;

#define SCENE_EXTENSION ".phys"
#define VORONOI_EXTENSION ".vdf"
#define SCENE_FOLDER_NAME "scenes"
#define VORONOI_FOLDER_NAME "voronoi_data"
namespace fs = std::filesystem;

static fs::path getProjectPath(std::string folderName)
{
    fs::path path = fs::current_path();
    if (path.filename() != folderName) {
        path = fs::current_path() / folderName;
        fs::create_directories(path);
    }
    return path;
}

std::string OpenFileDialog(const char* filter, const char* title)
{
    char filename[MAX_PATH] = "";
    OPENFILENAME ofn;
    ZeroMemory(&ofn, sizeof(ofn));

    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr; // Set this if you have a window handle
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
    ofn.lpstrTitle = title;

    if (GetOpenFileName(&ofn))
    {
        return std::string(filename);
    }
    return ""; // User canceled
}

std::string SaveFileDialog(const char* filter, const char* title, const char* defaultExt)
{
    char filename[MAX_PATH] = "";
    OPENFILENAME ofn;
    ZeroMemory(&ofn, sizeof(ofn));

    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
    ofn.lpstrDefExt = defaultExt;
    ofn.lpstrTitle = title;

    if (GetSaveFileName(&ofn))
    {
        return std::string(filename);
    }
    return ""; // User canceled
}

#include <shobjidl.h>  // For IFileDialog
std::string OpenFolderDialog(const std::wstring& title = L"Select Folder")
{
    std::string result;

    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    if (SUCCEEDED(hr))
    {
        IFileDialog* pFileDialog;
        hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_ALL,
            IID_IFileDialog, reinterpret_cast<void**>(&pFileDialog));

        if (SUCCEEDED(hr))
        {
            DWORD options;
            pFileDialog->GetOptions(&options);
            pFileDialog->SetOptions(options | FOS_PICKFOLDERS); // Important: Pick folders

            pFileDialog->SetTitle(title.c_str());

            if (SUCCEEDED(pFileDialog->Show(nullptr)))
            {
                IShellItem* pItem;
                if (SUCCEEDED(pFileDialog->GetResult(&pItem)))
                {
                    PWSTR path = nullptr;
                    if (SUCCEEDED(pItem->GetDisplayName(SIGDN_FILESYSPATH, &path)))
                    {
                        char buffer[MAX_PATH];
                        wcstombs_s(nullptr, buffer, path, MAX_PATH);
                        result = buffer;
                        CoTaskMemFree(path);
                    }
                    pItem->Release();
                }
            }
            pFileDialog->Release();
        }
        CoUninitialize();
    }
    return result;
}


class VoronoiParser
{
public:
    static void saveDiagram(const cVoronoiDiagram& diagram)
    {
        std::string filename = SaveFileDialog("Voronoi Files (*.vdf)\0*.vdf\0All Files (*.*)\0*.*\0", "Save Voronoi Diagram", "vdf");

        if (filename.empty())
        {
            std::cerr << "Voronoi diagram saving canceled.\n";
            return;
        }

        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Error: Could not open file for writing: " << filename << "\n";
            return;
        }


        // Write number of vertices
        size_t vertexCount = diagram.vertices.size();
        file.write(reinterpret_cast<char*>(&vertexCount), sizeof(vertexCount));

        // Write vertices
        for (const auto& vert : diagram.vertices)
        {
            file.write(reinterpret_cast<const char*>(&vert.site), sizeof(cVec2));

            size_t edgeCount = vert.edgeIndices.size();
            file.write(reinterpret_cast<const char*>(&edgeCount), sizeof(edgeCount));
            file.write(reinterpret_cast<const char*>(vert.edgeIndices.data()), edgeCount * sizeof(unsigned));

            size_t seedCount = vert.seedIndices.size();
            file.write(reinterpret_cast<const char*>(&seedCount), sizeof(seedCount));
            file.write(reinterpret_cast<const char*>(vert.seedIndices.data()), seedCount * sizeof(unsigned));
        }

        // Write number of edges
        size_t edgeCount = diagram.edges.size();
        file.write(reinterpret_cast<char*>(&edgeCount), sizeof(edgeCount));

        // Write edges
        for (const auto& edge : diagram.edges)
        {
            file.write(reinterpret_cast<const char*>(&edge.origin), sizeof(cVec2));
            file.write(reinterpret_cast<const char*>(&edge.endDir), sizeof(cVec2));
            file.write(reinterpret_cast<const char*>(&edge.infinite), sizeof(bool));
        }

        // Write number of v_points
        size_t pointCount = diagram.v_points.size();
        file.write(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));

        // Write v_points
        file.write(reinterpret_cast<const char*>(diagram.v_points.data()), pointCount * sizeof(cVec2));

        file.close();
    }

    static cVoronoiDiagram loadDiagram()
    {
        std::string filename = OpenFileDialog("Voronoi Files (*.vdf)\0*.vdf\0All Files (*.*)\0*.*\0", "Load Voronoi Diagram");

        if (filename.empty())
        {
            std::cerr << "Voronoi diagram loading canceled.\n";
            return cVoronoiDiagram();
        }

        std::ifstream file(filename, std::ios::binary);
        if (!file)
        {
            std::cerr << "Error: Could not open file for reading: " << filename << "\n";
            return cVoronoiDiagram();
        }

        cVoronoiDiagram result;
        // Read number of vertices
        size_t vertexCount;
        file.read(reinterpret_cast<char*>(&vertexCount), sizeof(vertexCount));
        result.vertices.resize(vertexCount);

        // Read vertices
        for (auto& vert : result.vertices)
        {
            file.read(reinterpret_cast<char*>(&vert.site), sizeof(cVec2));

            size_t edgeCount;
            file.read(reinterpret_cast<char*>(&edgeCount), sizeof(edgeCount));
            vert.edgeIndices.resize(edgeCount);
            file.read(reinterpret_cast<char*>(vert.edgeIndices.data()), edgeCount * sizeof(unsigned));

            size_t seedCount;
            file.read(reinterpret_cast<char*>(&seedCount), sizeof(seedCount));
            vert.seedIndices.resize(seedCount);
            file.read(reinterpret_cast<char*>(vert.seedIndices.data()), seedCount * sizeof(unsigned));
        }

        // Read number of edges
        size_t edgeCount;
        file.read(reinterpret_cast<char*>(&edgeCount), sizeof(edgeCount));
        result.edges.resize(edgeCount);

        // Read edges
        for (auto& edge : result.edges)
        {
            file.read(reinterpret_cast<char*>(&edge.origin), sizeof(cVec2));
            file.read(reinterpret_cast<char*>(&edge.endDir), sizeof(cVec2));
            file.read(reinterpret_cast<char*>(&edge.infinite), sizeof(bool));
        }

        // Read number of v_points
        size_t pointCount;
        file.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
        result.v_points.resize(pointCount);

        // Read v_points
        file.read(reinterpret_cast<char*>(result.v_points.data()), pointCount * sizeof(cVec2));

        file.close();

        return result;
    }

    static cVoronoiDiagram loadDiagramFromStream(std::ifstream& file)
    {
        if (!file)
        {
            std::cerr << "Error: Could not open file for reading " << std::endl;
            return cVoronoiDiagram();
        }

        cVoronoiDiagram result;
        // Read number of vertices
        size_t vertexCount;
        file.read(reinterpret_cast<char*>(&vertexCount), sizeof(vertexCount));
        result.vertices.resize(vertexCount);

        // Read vertices
        for (auto& vert : result.vertices)
        {
            file.read(reinterpret_cast<char*>(&vert.site), sizeof(cVec2));

            size_t edgeCount;
            file.read(reinterpret_cast<char*>(&edgeCount), sizeof(edgeCount));
            vert.edgeIndices.resize(edgeCount);
            file.read(reinterpret_cast<char*>(vert.edgeIndices.data()), edgeCount * sizeof(unsigned));

            size_t seedCount;
            file.read(reinterpret_cast<char*>(&seedCount), sizeof(seedCount));
            vert.seedIndices.resize(seedCount);
            file.read(reinterpret_cast<char*>(vert.seedIndices.data()), seedCount * sizeof(unsigned));
        }

        // Read number of edges
        size_t edgeCount;
        file.read(reinterpret_cast<char*>(&edgeCount), sizeof(edgeCount));
        result.edges.resize(edgeCount);

        // Read edges
        for (auto& edge : result.edges)
        {
            file.read(reinterpret_cast<char*>(&edge.origin), sizeof(cVec2));
            file.read(reinterpret_cast<char*>(&edge.endDir), sizeof(cVec2));
            file.read(reinterpret_cast<char*>(&edge.infinite), sizeof(bool));
        }

        // Read number of v_points
        size_t pointCount;
        file.read(reinterpret_cast<char*>(&pointCount), sizeof(pointCount));
        result.v_points.resize(pointCount);

        // Read v_points
        file.read(reinterpret_cast<char*>(result.v_points.data()), pointCount * sizeof(cVec2));

        file.close();

        return result;
    }
};

class SceneParser
{
	cFractureWorld* world;
	
    std::vector<std::string> tokenize(const std::string& line)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string token;
        while (ss >> token)
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    cVec2 parseVec2(const std::string& str)
    {
        float x, y;
        sscanf_s(str.c_str(), "(%f,%f)", &x, &y);
        return cVec2{ x, y };
    }

    std::vector<int> parseIntList(const std::string& str)
    {
        std::vector<int> indices;
        std::stringstream ss(str);
        std::string segment;
        while (std::getline(ss, segment, ']'))
        {
            if (segment.find('[') != std::string::npos)
            {
                size_t start = segment.find('[') + 1;
                size_t end = segment.find(']');
                std::string data = segment.substr(start, end - start);

                std::stringstream intStream(data);
                std::string num;
                while (std::getline(intStream, num, ','))
                {
                    indices.push_back(std::stoi(num));
                }
            }
        }
        return indices;
    }

    std::vector<cVec2> parseVertices(const std::string& str)
    {
        std::vector<cVec2> vertices;
        std::stringstream ss(str);
        std::string segment;
        while (std::getline(ss, segment, ']'))
        {
            if (segment.find('[') != std::string::npos)
            {
                size_t start = segment.find('[') + 1;
                size_t end = segment.find(']');
                std::string data = segment.substr(start, end - start);
                vertices.push_back(parseVec2(data));
            }
        }
        return vertices;
    }

    void processActor(std::ifstream& file)
    {
        ActorConfig config;
        bool isFracturable = false;
        cFractureMaterial fractureMaterial;
        int fracturePatternIndex = -1;

        std::string line;
        while (std::getline(file, line) && line.find("}") == std::string::npos)
        {
            auto tokens = tokenize(line);
            if (tokens.size() < 2) continue;

            if (tokens[0] == "type:")
            {
                if (tokens[1] == "DYNAMIC") config.type = DYNAMIC;
                else if (tokens[1] == "STATIC") config.type = STATIC;
                else if (tokens[1] == "KINEMATIC") config.type = KINEMATIC;
            }
            else if (tokens[0] == "position:") config.position = parseVec2(tokens[1]);
            else if (tokens[0] == "angle:") config.angle = std::stof(tokens[1]);
            else if (tokens[0] == "linearVelocity:") config.linearVelocity = parseVec2(tokens[1]);
            else if (tokens[0] == "angularVelocity:") config.angularVelocity = std::stof(tokens[1]);
            else if (tokens[0] == "gravityScale:") config.gravityScale = std::stof(tokens[1]);
            else if (tokens[0] == "linearDamping:") config.linearDamping = std::stof(tokens[1]);
            else if (tokens[0] == "angularDamping:") config.angularDamping = std::stof(tokens[1]);
            else if (tokens[0] == "fractureMaterial:")
            {
                isFracturable = true;
                processFractureMaterial(file, fractureMaterial);
            }
            else if (tokens[0] == "fracturePatternIndex:")
            {
                isFracturable = true;
                fracturePatternIndex = std::stoi(tokens[1]);
            }
        }

        int actorIndex = world->CreateActor(config);

        if (isFracturable)
        {
            int fractureIndex = world->MakeFracturable(actorIndex, fractureMaterial);
            if (fracturePatternIndex >= 0)
            {
                world->SetFracturePattern(fracturePatternIndex, fractureIndex);
            }
        }
    }

    void processShape(std::ifstream& file)
    {
        ShapeConfig config;
        int actorIndex = -1;
        std::string line;
        cPolygon poly;
        bool shapeSet = false;
        
        while (std::getline(file, line) && line.find("}") == std::string::npos)
        {
            auto tokens = tokenize(line);

            if (tokens.size() < 2) continue;

            if (tokens[0] == "actorIndex:") actorIndex = std::stoi(tokens[1]);
            else if (tokens[0] == "friction:") config.friction = std::stof(tokens[1]);
            else if (tokens[0] == "restitution:") config.restitution = std::stof(tokens[1]);
            else if (tokens[0] == "box:")
            {
                cVec2 halfExtents = parseVec2(tokens[1]);
                poly = GeomMakeBox(halfExtents.x, halfExtents.y);
                shapeSet = true;
            }
            else if (tokens[0] == "offsetBox:")
            {
                // Format: (hx, hy, (cx, cy), angle)
                float hx, hy, angle = 0.0f;
                cVec2 center;
                sscanf_s(tokens[1].c_str(), "(%f,%f,(%f,%f),%f)", &hx, &hy, &center.x, &center.y, &angle);
                poly = GeomMakeOffsetBox(hx, hy, center, angle);
                shapeSet = true;
            }
            else if (tokens[0] == "square:")
            {
                float h = std::stof(tokens[1]);
                poly = GeomMakeSquare(h);
                shapeSet = true;
            }
            else if (tokens[0] == "regular:")
            {
                int count = std::stoi(tokens[1]);
                poly = GeomMakeRegularPolygon(count);
                shapeSet = true;
            }
            else if (tokens[0] == "vertices:")
            {
                auto verts = parseVertices(line);
                poly = cPolygon{ verts.data(), static_cast<int>(verts.size()) };
                shapeSet = true;
            }
        }

        if (actorIndex >= 0)
        {
            cassert(shapeSet);
            world->CreateShape(actorIndex, config, &poly);
        }
    }

    void processFractureMaterial(std::ifstream& file, cFractureMaterial& material)
    {
        std::string line;
        while (std::getline(file, line) && line.find("}") == std::string::npos)
        {
            auto tokens = tokenize(line);
            if (tokens.size() < 2) continue;

            if (tokens[0] == "toughness:") material.toughness = std::stof(tokens[1]);
            else if (tokens[0] == "elasticity:") material.elasticity = std::stof(tokens[1]);
            else if (tokens[0] == "brittleness:") material.brittleness = std::stof(tokens[1]);
            else if (tokens[0] == "anisotropy:") material.anisotropy = parseVec2(tokens[1]);
            else if (tokens[0] == "anisotropyFactor:") material.anisotropyFactor = std::stof(tokens[1]);
            else if (tokens[0] == "k:") material.k = std::stof(tokens[1]);
        }
    }

    void processVDFList(std::ifstream& file)
    {
        std::string folderPath = OpenFolderDialog(L"Select Folder Containing VDF Files");
        if (folderPath.empty())
        {
            std::cerr << "VDF folder selection canceled.\n";
            return;
        }

        std::filesystem::path vdfPath(folderPath); // No need to parent_path anymore

        // Multiline block parsing
        std::string line;
        std::string combinedList;
        while (std::getline(file, line))
        {
            // Remove comments or trailing whitespace
            line = line.substr(0, line.find("//")); // strip inline comments
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());

            if (line.empty())
                continue;

            if (line.find('{') != std::string::npos)
                continue; // Skip line with just '{'

            if (line.find('}') != std::string::npos)
                break; // End of block

            combinedList += line; // Accumulate lines like: "trapezoid.vdf,x.vdf"
        }

        // Split by commas
        std::stringstream ss(combinedList);
        std::string filename;

        while (std::getline(ss, filename, ','))
        {
            // Trim whitespace
            filename.erase(0, filename.find_first_not_of(" \t"));
            filename.erase(filename.find_last_not_of(" \t") + 1);

            if (filename.empty()) continue;

            auto fullPath = vdfPath / filename;

            std::ifstream vfile(fullPath, std::ios::binary);
            if (!vfile)
            {
                std::cerr << "Failed to open VDF: " << fullPath << "\n";
                continue;
            }

            cVoronoiDiagram diagram = VoronoiParser::loadDiagramFromStream(vfile);
            vfile.close();
            cAABB aabb{ {-350,-350} , {350,350} };
            world->CreateNewFracturePattern(diagram, aabb); // TODO: Add bounds
            std::cout << "Loaded VDF pattern: " << filename << "\n";
        }
    }


public:
    explicit SceneParser(cFractureWorld* inWorld) : world(inWorld) {}

    void SceneParser::loadFromFile()
    {
        std::string filename = OpenFileDialog("Scene Files (*.phys)\0*.phys\0All Files (*.*)\0*.*\0", "Load Physics Scene");

        if (filename.empty())
        {
            std::cerr << "Scene loading canceled.\n";
            return;
        }

        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        // First pass – look for VDF block
        std::string line;
        while (std::getline(file, line))
        {
            if (line.find("VDF {") != std::string::npos)
            {
                processVDFList(file);
                break; // Only one VDF block supported
            }
        }

        // Second pass – rewind and load actors/shapes
        file.clear(); // Clear EOF flag
        file.seekg(0); // Go back to start

        while (std::getline(file, line))
        {
            if (line.find("Actor {") != std::string::npos)
            {
                processActor(file);
            }
            else if (line.find("Shape {") != std::string::npos)
            {
                processShape(file);
            }
        }

        file.close();
        std::cout << "Loaded Scene: " << filename << std::endl;
    }

};

