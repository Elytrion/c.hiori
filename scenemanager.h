#pragma once


class DebugGraphics;
class UIManager;

struct PhysicsSceneSettings
{
	float physicsStepTime{ 0.0167f };
	int primaryIterations{ 4 };
	int secondaryIterations{ 2 };
	bool drawShapes{ true };
	bool drawAABBs{ false };
	bool drawTreeAABBs{ false };
	bool drawMass{ true };
	bool drawContactPoints{ false };
	bool drawContactNormals{ false };
	bool drawContactImpulses{ false };
	bool drawFrictionImpulses{ false };
	bool drawCenterOfMasses{ false };
	bool runBasicSolver{ false };
	bool warmStart{ true };
};

class PhysicsScene
{
protected:
	DebugGraphics* drawer{ nullptr };
	void* world{ nullptr };
	float accumulator{ 0.0f };
public:
	float currentZoom{ 0.0f };
	PhysicsSceneSettings settings{};
	PhysicsScene(DebugGraphics* drawer, void* world) : drawer(drawer), world(world), accumulator(0.0f) {}
	
	virtual void Load() = 0;
	virtual void Unload();
	virtual void Update(float dt);
	void Step(float fdt);
};

class SceneManager
{
private:
	DebugGraphics* drawer{ nullptr };
	UIManager* uimanager{ nullptr };
	void* world{ nullptr };
	std::vector<PhysicsScene*> scenes{};
	
	void* voronoi{ nullptr };
	
	bool isPaused{ true };
	bool stepOnce{ false };
public:

	bool drawCamera{ true };
	bool drawInstructions{ true };
	bool drawStats{ true };
	bool drawUI{ true };
	bool drawAABBVoronoi{ false };
	bool inVoronoiEditor{ false };
	bool loadedUI{ false };

	int currentScene{ -1 };
	int sceneCount{ 0 };

	SceneManager(DebugGraphics* drawer, UIManager* uimanager, void* world, void* vd);
	~SceneManager();
	
	void ChangeScene(int sceneIndex);
	void Update(float dt);
	void UpdatePhysics(float dt);
	void UpdateVoronoi(float dt);
	void AddScene(PhysicsScene* scene);
	void RemoveScene(int sceneIndex);
	void HandleCameraInput();
	PhysicsScene* GetCurrentScene();

	void LoadPhysicsUI();
	void LoadVoronoiUI();
};