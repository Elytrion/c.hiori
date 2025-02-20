#pragma once
#include "voronoi.h"

class DebugGraphics;

class VoronoiScene
{
protected:
	DebugGraphics* drawer{ nullptr };
	chiori::cVoronoiDiagram voronoi;
	std::vector<std::vector<chiori::cVec2>> tris;

public:
	VoronoiScene(DebugGraphics* _drawer) : drawer{ _drawer } {}
	bool drawTriangles = false;
	bool drawVoronoi = true;
	bool drawPoints = true;
	bool drawInfinite = true;

	virtual void Load() = 0;
	virtual void Update(float dt);
	virtual void Unload();
};

class VoronoiSceneManager
{
	DebugGraphics* drawer{ nullptr };
	std::vector<VoronoiScene*> scenes{};
	
public:
	int currentScene{ -1 };
	int sceneCount{ 0 };
	
	VoronoiSceneManager(DebugGraphics* drawer);
	~VoronoiSceneManager();

	void ChangeScene(int sceneIndex);
	void Update(float dt);
	void AddScene(VoronoiScene* scene);
	void RemoveScene(int sceneIndex);
	VoronoiScene* GetCurrentScene();
};