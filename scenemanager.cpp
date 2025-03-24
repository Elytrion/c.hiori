#include "pch.h"
#include "scenemanager.h"
#include "fractureWorld.h"
#include "graphics.h"
#include "cprocessing.h"
#include "uimanager.h"
#include "parser.hpp"

using namespace chiori;

void PhysicsScene::Unload()
{
	cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);
	pWorld->f_patterns.Clear();
	int capacity = pWorld->p_actors.capacity();
	for (int i = capacity - 1; i >= 0; i--)
	{
		if (pWorld->p_actors.isValid(i))
		{
			int index = pWorld->IsFracturable(i);
			if (index >= 0)
			{
				pWorld->MakeUnfracturable(index);
			}
			pWorld->RemoveActor(i);
		}
	}
}

void PhysicsScene::Update(float dt)
{
	accumulator += dt;
	const float maxAccumulator = settings.physicsStepTime * 5;
	if (accumulator > maxAccumulator)
	{
		accumulator = maxAccumulator;
	}

	while (accumulator >= settings.physicsStepTime)
	{
		// Call the physics step function with the fixed time step
		Step(settings.physicsStepTime);
		// Subtract the fixed step time from the accumulator
		accumulator -= settings.physicsStepTime;
	}
}

void PhysicsScene::Step(float fdt)
{
	cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);
	pWorld->runBasicSolver = settings.runBasicSolver;
	pWorld->f_step(
		settings.physicsStepTime,
		settings.primaryIterations,
		settings.secondaryIterations,
		settings.warmStart
	);
}

#pragma region Scenes
// loads in a default scene of 2 stacked boxes and a floor
// Shows basic physics simulation with collision and gravity
class DefaultScene : public PhysicsScene
{
public:
	DefaultScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.type = cActorType::STATIC;
		int floorID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		s_config.friction = 0.2f;
		cPolygon floorShape = GeomMakeBox(10.0f, 0.25f);
		int floorShapeIndex = pWorld->CreateShape(floorID, s_config, &floorShape);

		// Create the ground box
		a_config.type = cActorType::DYNAMIC;
		a_config.position = { 0.0f, 1.0f };
		int boxID = pWorld->CreateActor(a_config);
		cPolygon box = GeomMakeBox(1.0f, 1.0f);
		int boxShapeIndex = pWorld->CreateShape(boxID, s_config, &box);

		// Create the top box
		a_config.position = { 0.25f, 3.5f };
		int boxID2 = pWorld->CreateActor(a_config);
		int boxShapeIndex2 = pWorld->CreateShape(boxID2, s_config, &box);

		// configure camera
		{
			currentZoom = 80.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 5.0f });
		}
	}
};
// Loads in a scene with slanted ramps and 5 boxes of varying friction
// Boxes with high friction should be able to stop on ramped surfaces
// Boxes with low friction should slide down the ramps
class RampScene : public PhysicsScene
{
public:
	RampScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		a_config.position = { 0.0f, 10.0f };
		a_config.angle = -0.25f;
		int staticID2 = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		s_config.friction = 0.2f;
		cPolygon floorShape = GeomMakeBox(15.0f, 0.25f);
		int floorShapeIndex = pWorld->CreateShape(staticID, s_config, &floorShape);

		cPolygon ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -3.0f, 12.0f }, -0.25f);
		pWorld->CreateShape(staticID, s_config, &ramp);

		ramp = GeomMakeOffsetBox(0.1f, 0.75f, { 4.5f, 10.3f });
		pWorld->CreateShape(staticID, s_config, &ramp);

		ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -1.0f, 7.0f }, 0.25f);
		pWorld->CreateShape(staticID, s_config, &ramp);

		ramp = GeomMakeOffsetBox(0.1f, 0.75f, { -8.3f, 5.5f });
		pWorld->CreateShape(staticID, s_config, &ramp);

		ramp = GeomMakeOffsetBox(6.0f, 0.25f, { -3.0f, 2.0f }, -0.25f);
		pWorld->CreateShape(staticID, s_config, &ramp);

		cPolygon box = GeomMakeBox(0.3f, 0.3f);
		s_config.density = 25.0f;
		float friction[5] = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

		for (int i = 0; i < 5; ++i)
		{
			ActorConfig box_config;
			box_config.type = cActorType::DYNAMIC;
			box_config.position = { -8.0f + 2.0f * i, 14.0f };
			int bodyId = pWorld->CreateActor(box_config);

			s_config.friction = friction[i];
			pWorld->CreateShape(bodyId, s_config, &box);
		}


		{
			currentZoom = 45.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 7.0f });
		}
	}
};
// Loads in a scene with a row of 15 dominos, with the first one set to fall
// The dominos should knock each other over, then at the end, 
// the last domino should fall in a way to push all the previous dominos flat (double-domino effect)
class DominoScene : public PhysicsScene
{
public:
	DominoScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.position = { 0.0f, -1.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		cPolygon floorShape = GeomMakeBox(20.0f, 1.0f);
		pWorld->CreateShape(staticID, s_config, &floorShape);

		// Create row of dominos
		cPolygon box = GeomMakeBox(0.125f, 0.5f);
		s_config.friction = 0.6f;
		a_config.type = cActorType::DYNAMIC;

		int count = 17;
		float x = -0.5f * count;
		for (int i = 0; i < count; ++i)
		{
			a_config.position = { x, 0.5f };
			int bodyId = pWorld->CreateActor(a_config);
			pWorld->CreateShape(bodyId, s_config, &box);
			if (i == 0)
			{
				cActor* actor = pWorld->p_actors[bodyId];
				actor->applyImpulse({ 0.2f, 0.0f }, { x, 1.0f });
			}

			x += 1.0f;
		}

		{
			currentZoom = 75.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 4.0f });
		}
	}
};
// Loads in a scene of a pyramid of boxes that are intially overlapping
// The scene should recover from the overlap and stabilize
class OverlapRecoveryScene : public PhysicsScene
{
public:
	OverlapRecoveryScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.position = { 0.0f, -1.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);
		ShapeConfig s_config;
		cPolygon floorShape = GeomMakeBox(10.0f, 1.0f);
		pWorld->CreateShape(staticID, s_config, &floorShape);


		int baseCount = 4;
		float overlap = 0.25f;
		float extent = 0.5f;
		float fraction = 1.0f - overlap;
		float y = 0.3f;

		a_config.type = cActorType::DYNAMIC;
		cPolygon box = GeomMakeBox(extent, extent);

		for (int i = 0; i < baseCount; ++i)
		{
			float x = fraction * extent * (i - baseCount);
			for (int j = i; j < baseCount; ++j)
			{
				a_config.position = { x, y };
				int bodyId = pWorld->CreateActor(a_config);

				pWorld->CreateShape(bodyId, s_config, &box);

				x += 2.0f * fraction * extent;
			}

			y += 2.0f * fraction * extent;
		}
	}
};
// Loads in a scene with a stack of 6 boxes on one side
// and a Tower of Lire of 6 boxes on the other
// The scene should demonstrate the stability of the physics simulation
class StackScene : public PhysicsScene
{
public:
	StackScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.position = { 0.0f, -1.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		cPolygon floorShape = GeomMakeBox(20.0f, 1.0f);
		pWorld->CreateShape(staticID, s_config, &floorShape);

		// Create the stack of 5 boxes on one side
		cPolygon boxShape = GeomMakeBox(0.5f, 0.5f); // Box dimensions: 1x1
		s_config.friction = 0.5f;
		int boxCount = 6;
		a_config.type = cActorType::DYNAMIC;
		float xStack = -5.0f; // Position of the stack
		float yStack = 1.0f;  // Start from the ground

		for (int i = 0; i < boxCount; ++i)
		{
			a_config.position = { xStack, yStack };
			int bodyId = pWorld->CreateActor(a_config);
			pWorld->CreateShape(bodyId, s_config, &boxShape);

			yStack += 1.0f; // Stack each box 1 unit higher
		}

		// Create the tower of Lire (block-stacking problem) on the other side
		float xTower = 5.0f; // Position of the tower
		float yTower = 0.0f + 1.0f * boxCount; // Start from the top

		for (int i = 0; i < boxCount; ++i)
		{
			float offset = 1.0f / (2.0f * (i + 1)); // Offset for overhang, largest at the top

			a_config.position = { xTower + offset, yTower };
			int bodyId = pWorld->CreateActor(a_config);
			pWorld->CreateShape(bodyId, s_config, &boxShape);

			yTower -= 1.0f; // Raise the next box
		}

		// Configure the camera to view both setups
		{
			currentZoom = 50.0f; // Adjust zoom to fit both setups
			drawer->ChangeZoom(currentZoom);

			// Center the camera between the two setups, with a slight upward adjustment
			drawer->SetCamera({ 0.0f, 2.5f });
		}
	}
};
// Loads in a scene with an arch held up by friction and their contact points
// The scene should further demonstrate the stability of the physics simulation
class ArchScene : public PhysicsScene
{
public:
	ArchScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Configure camera
		{
			currentZoom = 35.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 8.0f });
		}

		// Define polygon points for the arch
		cVec2 ps1[9] = { {16.0f, 0.0f},
						{14.93803712795643f, 5.133601056842984f},
						{13.79871746027416f, 10.24928069555078f},
						{12.56252963284711f, 15.34107019122473f},
						{11.20040987372525f, 20.39856541571217f},
						{9.66521217819836f, 25.40369899225096f},
						{7.87179930638133f, 30.3179337000085f},
						{5.635199558196225f, 35.03820717801641f},
						{2.405937953536585f, 39.09554102558315f} };

		cVec2 ps2[9] = { {24.0f, 0.0f},
						{22.33619528222415f, 6.02299846205841f},
						{20.54936888969905f, 12.00964361211476f},
						{18.60854610798073f, 17.9470321677465f},
						{16.46769273811807f, 23.81367936585418f},
						{14.05325025774858f, 29.57079353071012f},
						{11.23551045834022f, 35.13775818285372f},
						{7.752568160730571f, 40.30450679009583f},
						{3.016931552701656f, 44.28891593799322f} };

		// Scale the points
		float scale = 0.25f;
		for (int i = 0; i < 9; ++i)
		{
			ps1[i] = ps1[i] * scale;
			ps2[i] = ps2[i] * scale;
		}


		// Create a floor
		ActorConfig a_config;
		a_config.position = { 0.0f, -2.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		s_config.friction = 0.6f;
		cPolygon floorShape = GeomMakeBox(100.0f, 1.0f);
		pWorld->CreateShape(staticID, s_config, &floorShape);

		a_config.position = { 0.0f, 0.0f };
		// Create left side of the arch
		ActorConfig dynamicConfig;
		dynamicConfig.type = cActorType::DYNAMIC;
		for (int i = 0; i < 8; ++i)
		{
			cVec2 ps[4] = { ps1[i], ps2[i], ps2[i + 1], ps1[i + 1] };
			cPolygon polygon{ ps, 4 };
			dynamicConfig.position = { 0.0f, 0.0f };
			int bodyID = pWorld->CreateActor(dynamicConfig);

			pWorld->CreateShape(bodyID, s_config, &polygon);
		}

		// Create right side of the arch
		for (int i = 0; i < 8; ++i)
		{
			cVec2 ps[4] = {
				{-ps2[i].x, ps2[i].y}, {-ps1[i].x, ps1[i].y}, {-ps1[i + 1].x, ps1[i + 1].y}, {-ps2[i + 1].x, ps2[i + 1].y} };
			cPolygon polygon{ ps, 4 };
			dynamicConfig.position = { 0.0f, 0.0f };
			int bodyID = pWorld->CreateActor(dynamicConfig);

			pWorld->CreateShape(bodyID, s_config, &polygon);
		}

		// Create the top of the arch
		{
			cVec2 ps[4] = { ps1[8], ps2[8], {-ps2[8].x, ps2[8].y}, {-ps1[8].x, ps1[8].y} };
			cPolygon polygon{ ps, 4 };
			dynamicConfig.position = { 0.0f, 0.0f };
			int bodyID = pWorld->CreateActor(dynamicConfig);

			pWorld->CreateShape(bodyID, s_config, &polygon);
		}

	}
};
// Loads in a scene with a heap of polygons of varying sizes
// Pressing R will apply a random force to a random polygon
// This showcases the engine can support more complex shapes
class PolygonScene : public PhysicsScene
{
public:
	PolygonScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a confining box
		ActorConfig a_config;
		a_config.position = { 0.0f, -1.0f };
		a_config.type = cActorType::STATIC;
		int staticID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		cPolygon wallShape = GeomMakeOffsetBox(15.0f, 0.25f, { 0.0f, 0.0f });
		pWorld->CreateShape(staticID, s_config, &wallShape);

		wallShape = GeomMakeOffsetBox(15.0f, 0.25f, { 0.0f, 30.0f });
		pWorld->CreateShape(staticID, s_config, &wallShape);

		wallShape = GeomMakeOffsetBox(0.25f, 15.0f, { 15.25f, 15.0f });
		pWorld->CreateShape(staticID, s_config, &wallShape);

		wallShape = GeomMakeOffsetBox(0.25f, 15.0f, { -15.25f, 15.0f });
		pWorld->CreateShape(staticID, s_config, &wallShape);


		a_config.type = cActorType::DYNAMIC;
		// create a heap of polygons
		for (int iy = 1; iy < 5; ++iy)
		{
			for (int i = 3; i < 8; ++i)
			{
				cPolygon poly = GeomMakeRegularPolygon(i);

				a_config.position = { -5.0f + 1.0f * i, iy * 2.0f };

				int bodyID = pWorld->CreateActor(a_config);
				pWorld->CreateShape(bodyID, s_config, &poly);
			}
		}
	}

	void Update(float dt) override
	{
		PhysicsScene::Update(dt);

		// Pick a random polygon and apply a random force
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);
		if (CP_Input_KeyTriggered(KEY_R))
		{
			int bodyID = -1;
			while (!pWorld->p_actors.isValid(bodyID))
			{
				bodyID = CP_Random_RangeInt(0, pWorld->p_actors.capacity() - 1);
			}
			cActor* actor = pWorld->p_actors[bodyID];

			float xforce = CP_Random_RangeFloat(-100.0f, 100.0f);
			float yforce = CP_Random_RangeFloat(-100.0f, 100.0f);

			cVec2 force = { xforce, yforce };
			actor->applyImpulse(force, actor->position);
		}
	}

};

class SelfBalancingBoxScene : public PhysicsScene
{
	int boxID{ -1 };
	const cManifoldPoint* mp = nullptr;
public:
	SelfBalancingBoxScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.type = cActorType::STATIC;
		int floorID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		s_config.friction = 0.2f;
		cPolygon floorShape = GeomMakeBox(10.0f, 0.25f);
		pWorld->CreateShape(floorID, s_config, &floorShape);

		// Create Walls
		int wallID = pWorld->CreateActor(a_config);
		cPolygon wallShape = GeomMakeOffsetBox(0.25f, 10.0f, { -7.0f, 0.0f });
		pWorld->CreateShape(wallID, s_config, &wallShape);
		wallShape = GeomMakeOffsetBox(0.25f, 10.0f, { 7.0f, 0.0f });
		pWorld->CreateShape(wallID, s_config, &wallShape);

		// Create the ground box
		a_config.type = cActorType::DYNAMIC;
		a_config.position = { 0.0f, 1.5f };
		a_config.angle = 45.1f * DEG2RAD;
		a_config.angularDamping = 0.75f;
		boxID = pWorld->CreateActor(a_config);
		cPolygon box = GeomMakeBox(0.5f, 0.5f);

		int boxShapeIndex = pWorld->CreateShape(boxID, s_config, &box);

		// configure camera
		{
			currentZoom = 90.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 3.0f });
		}
	}

	void Update(float dt) override
	{
		PhysicsScene::Update(dt);

		// Apply a force to the box to keep it balanced on its tip everytime it collides with the ground on one side
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);
		cActor* boxActor = pWorld->p_actors[boxID];
		if (boxActor)
		{
			cTransform xf = boxActor->getTransform();
			if (boxActor->contactCount < 1)
				return;


			int contactKey = boxActor->contactList;
			// cVec2 worldPoint = cTransformVec(xf, p.localAnchorA);
			while (contactKey != NULL_INDEX)
			{
				cContact* contact = pWorld->p_contacts[(contactKey >> 1)];
				const cManifold& manifold = contact->manifold;
				if (manifold.pointCount > 1)
				{
					if (!mp)
					{
						for (int i = 0; i < manifold.pointCount; ++i)
						{
							const cManifoldPoint* p = manifold.points + i;
							if (p->separation > commons::LINEAR_SLOP)
							{
								mp = p;
								break;
							}
						}
					}
					else
					{
						if (mp->separation <= commons::LINEAR_SLOP)
						{
							cVec2 worldPoint = cTransformVec(xf, mp->localAnchorA);
							cVec2 force = cVec2::up * 1.9f;
							boxActor->applyImpulse(force, worldPoint);
							mp = nullptr;
						}
					}
				}
				contactKey = contact->edges[contactKey & 1].nextKey;
			}
		}
	}
};

class FractureTestScene : public PhysicsScene
{
public:
	FractureTestScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		// Create a floor
		ActorConfig a_config;
		a_config.type = cActorType::STATIC;
		int floorID = pWorld->CreateActor(a_config);

		ShapeConfig s_config;
		s_config.friction = 0.2f;
		cPolygon floorShape = GeomMakeBox(10.0f, 0.25f);
		pWorld->CreateShape(floorID, s_config, &floorShape);

		// Create Walls
		int wallID = pWorld->CreateActor(a_config);
		cPolygon wallShape = GeomMakeOffsetBox(0.25f, 10.0f, { -7.0f, 0.0f });
		pWorld->CreateShape(wallID, s_config, &wallShape);
		wallShape = GeomMakeOffsetBox(0.25f, 10.0f, { 7.0f, 0.0f });
		pWorld->CreateShape(wallID, s_config, &wallShape);

		// Create the ground box
		a_config.type = cActorType::DYNAMIC;
		a_config.position = { 0.0f, 3.0f };
		a_config.angle = 45.1f * DEG2RAD;
		a_config.angularDamping = 0.75f;
		int boxID = pWorld->CreateActor(a_config);
		cPolygon box = GeomMakeBox(0.5f, 0.5f);
		int boxShapeIndex = pWorld->CreateShape(boxID, s_config, &box);
		cFractureMaterial fmat;
		fmat.k = 0.0f;
		pWorld->MakeFracturable(boxID, fmat);

		a_config.position = { 3.0f, 3.0f };
		int box2ID = pWorld->CreateActor(a_config);
		int boxShape2Index = pWorld->CreateShape(box2ID, s_config, &box);
		pWorld->MakeFracturable(box2ID, fmat);

		// configure camera
		{
			currentZoom = 90.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 3.0f });
		}
	}
};

class CustomScene : public PhysicsScene
{
public:
	CustomScene(DebugGraphics* drawer, void* world) : PhysicsScene(drawer, world) {}

	void Load() override
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		SceneParser parser(pWorld);
		parser.loadFromFile();

		{
			currentZoom = 90.0f;
			drawer->ChangeZoom(currentZoom);
			drawer->SetCamera({ 0.0f, 3.0f });
		}
	}
};
#pragma endregion

#pragma region Scene Manager
SceneManager::SceneManager(DebugGraphics* drawer, UIManager* uimanager, void* world, void* vd)
	: drawer(drawer), uimanager(uimanager), world(world), voronoi(vd)
{
	AddScene(new CustomScene(drawer, world));

	AddScene(new PolygonScene(drawer, world));
	AddScene(new RampScene(drawer, world));

	AddScene(new StackScene(drawer, world));
	AddScene(new FractureTestScene(drawer, world));
	//AddScene(new DefaultScene(drawer, world));

	AddScene(new DominoScene(drawer, world));
	//AddScene(new ArchScene(drawer, world));
	AddScene(new OverlapRecoveryScene(drawer, world));


	ChangeScene(0);
}

SceneManager::~SceneManager()
{
	for (int i = scenes.size() - 1; i >= 0; i--)
	{
		delete scenes[i];
	}
}

void SceneManager::ChangeScene(int sceneIndex)
{
	bool prevPaused = isPaused;
	isPaused = true;
	if (currentScene >= 0)
	{
		scenes[currentScene]->Unload();
	}
	currentScene = sceneIndex;
	scenes[currentScene]->Load();
	isPaused = prevPaused;
}

void SceneManager::Update(float dt)
{
	HandleCameraInput();

	if (CP_Input_KeyTriggered(KEY_I))
	{
		drawInstructions = !drawInstructions;
	}
	if (CP_Input_KeyTriggered(KEY_C))
	{
		drawCamera = !drawCamera;
	}
	if (CP_Input_KeyTriggered(KEY_S))
	{
		drawStats = !drawStats;
	}
	if (CP_Input_KeyTriggered(KEY_U))
	{
		drawUI = !drawUI;
	}

	if (inVoronoiEditor)
	{
		if (!loadedUI)
		{
			uimanager->ClearUI();
			drawer->ClearUI();
			LoadVoronoiUI();
		}
		UpdateVoronoi(dt);
	}
	else
	{
		if (!loadedUI)
		{
			uimanager->ClearUI();
			drawer->ClearUI();
			LoadPhysicsUI();
		}
		UpdatePhysics(dt);
	}

	if (drawUI)
	{
		drawer->DrawUI();
		uimanager->Update();
	}
}

void SceneManager::UpdatePhysics(float dt)
{
	if (CP_Input_KeyTriggered(KEY_EQUAL))
	{
		int index = (currentScene + 1) % sceneCount;
		ChangeScene(index);
		return;
	}

	if (!isPaused)
	{
		scenes[currentScene]->Update(CP_System_GetDt());
		if (CP_Input_KeyTriggered(KEY_SPACE))
			isPaused = true;
	}
	else
	{
		if (CP_Input_KeyTriggered(KEY_SPACE))
			isPaused = false;

		if (CP_Input_KeyTriggered(KEY_SLASH) || CP_Input_KeyDown(KEY_PERIOD))
			stepOnce = true;

		if (stepOnce)
		{
			scenes[currentScene]->Step(scenes[currentScene]->settings.physicsStepTime);
			stepOnce = false;
		}
	}

	if (CP_Input_KeyTriggered(KEY_F1))
	{
		if (!isPaused)
		{
			std::cout << "Cannot change solvers while unpaused!\n" << std::endl;
		}
		else
		{
			scenes[currentScene]->settings.runBasicSolver = !scenes[currentScene]->settings.runBasicSolver;
		}
	}
	if (CP_Input_KeyTriggered(KEY_V))
	{
		loadedUI = false;
		inVoronoiEditor = true;
	}
	if (CP_Input_KeyTriggered(KEY_ESCAPE))
	{
		// reload the current scene
		ChangeScene(currentScene);
	}

	drawer->DrawPhysicsWorld(world);
	cVec2 displayDim = drawer->getScreenDimensions();
	// draw camera
	if (drawCamera)
	{
		cFractureWorld* pWorld = static_cast<cFractureWorld*>(world);

		char buffer[64];
		CP_Color textColor = CP_Color{ 50, 255, 50, 255 };
		snprintf(buffer, 64, "Cam World Pos: (%.2f, %.2f)", drawer->getCameraWorldPos().x, drawer->getCameraWorldPos().y);
		drawer->DrawUIText(displayDim.x - 200, displayDim.y - 20, buffer, 15, textColor);
		snprintf(buffer, 64, "Cam Zoom: (%.2f)", drawer->getZoom());
		drawer->DrawUIText(displayDim.x - 200, displayDim.y - 40, buffer, 15, textColor);

		// draw 2 lines +x +y from camera pos
		cVec2 camScreenPos = drawer->getCameraScreenPos();
		cVec2 xLineEnd = camScreenPos + cVec2::right * 10.0f;
		cVec2 yLineEnd = camScreenPos + -cVec2::up * 10.0f;
		drawer->DrawUILine(camScreenPos.x, camScreenPos.y, xLineEnd.x, xLineEnd.y, CP_Color{ 255, 0, 100, 255 });
		drawer->DrawUILine(camScreenPos.x, camScreenPos.y, yLineEnd.x, yLineEnd.y, CP_Color{ 100, 255, 0, 255 });
	}

	if (drawInstructions)
	{
		char buffer[64];
		CP_Color textColor = CP_Color{ 50, 255, 50, 255 };
		float x = displayDim.x - 220;
		float y = 20;
		snprintf(buffer, 64, "Press SPACE to %s", isPaused ? "Play" : "Pause");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press SLASH to step once");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Hold PERIOD to keep stepping");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press EQUAL to change scenes");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press ESC to reset scene");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press LEFT/RIGHT BRACKET to zoom");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press UP/DOWN/LEFT/RIGHT to pan");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press I to toggle instructions");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press C to toggle camera display");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press S to toggle stats");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press U to toggle UI");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press F1 to toggle solvers");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;

		snprintf(buffer, 64, "Press V to edit patterns");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;
	}

	if (drawStats)
	{
		char buffer[64];
		CP_Color textColor = CP_Color{ 255, 50, 50, 255 };
		snprintf(buffer, 64, "Running %s", scenes[currentScene]->settings.runBasicSolver ? "PGS Basic" : "PGS Soft");
		drawer->DrawUIText(20, displayDim.y - 20, buffer, 15, textColor);

		int actorCapacity = static_cast<cFractureWorld*>(world)->p_actors.capacity();
		int actorCount = static_cast<cFractureWorld*>(world)->p_actors.size();
		snprintf(buffer, 64, "Actors: %d/%d", actorCount, actorCapacity);
		drawer->DrawUIText(20, displayDim.y - 40, buffer, 15, textColor);

		int shapeCapacity = static_cast<cFractureWorld*>(world)->p_shapes.capacity();
		int shapeCount = static_cast<cPhysicsWorld*>(world)->p_shapes.size();
		snprintf(buffer, 64, "Shapes: %d/%d", shapeCount, shapeCapacity);
		drawer->DrawUIText(20, displayDim.y - 60, buffer, 15, textColor);

		CP_Settings_TextSize(20);
		CP_Settings_BlendMode(CP_BLEND_ALPHA);
		CP_Settings_Fill(CP_Color_Create(0, 0, 0, 128));
		CP_Settings_NoStroke();
		CP_Graphics_DrawRect(0, 0, 150, 30);
		CP_Settings_Fill(CP_Color_Create(255, 255, 255, 255));
		char nbuffer[100];
		sprintf_s(nbuffer, 100, "FPS: %f", CP_System_GetFrameRate());
		CP_Font_DrawText(nbuffer, 20, 20);
	}
}

static void DrawVoronoiDiagram(const cVoronoiDiagram& voronoi)
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
				CP_Settings_Stroke(CP_Color_Create(255, 0, 255, 255)); // Purple for infinite edges
				CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);

				continue;
			}
			const cVec2& p1 = edge.origin;
			const cVec2& p2 = edge.endDir;
			CP_Settings_StrokeWeight(1);
			CP_Settings_Stroke(CP_Color_Create(255, 255, 255, 255)); // white for Delaunay edges
			CP_Graphics_DrawLine(p1.x, p1.y, p2.x, p2.y);
		}
	}

	for (const auto& p : voronoi.v_points)
	{
		CP_Settings_Fill(CP_Color_Create(255, 255, 0, 255));
		CP_Settings_Stroke(CP_Color_Create(255, 255, 0, 255));
		CP_Graphics_DrawCircle(p.x, p.y, 5);
	}
}

cVec2 originalPoint{ cVec2::zero };
cVec2 selectedPoint{ cVec2::zero };
float baseTimer = 0.05f;
float timer = 0.0f;
void SceneManager::UpdateVoronoi(float dt)
{
	cVoronoiDiagram* vd = static_cast<cVoronoiDiagram*>(voronoi);
	cVoronoiDiagram& vdiagram = *vd;
	DrawVoronoiDiagram(vdiagram);
	
	if (CP_Input_KeyTriggered(KEY_V))
	{
		loadedUI = false;
		inVoronoiEditor = false;
	}

	
	cVec2 displayDim = drawer->getScreenDimensions();
	if (drawInstructions)
	{
		char buffer[64];
		CP_Color textColor = CP_Color{ 50, 255, 50, 255 };
		float x = displayDim.x - 220;
		float y = 20;
		snprintf(buffer, 64, "Press F1 to show cutting AABB");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;
		snprintf(buffer, 64, "Press A to add a point");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;
		snprintf(buffer, 64, "Press D to remove a point");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;
		snprintf(buffer, 64, "Hold RMB to move a point");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;
		snprintf(buffer, 64, "Press V to close editor");
		drawer->DrawUIText(x, y, buffer, 15, textColor);
		y += 20;
	}

	if (CP_Input_KeyTriggered(KEY_F1))
	{
		drawAABBVoronoi = !drawAABBVoronoi;
	}

	if (drawAABBVoronoi)
	{
		cVec2 middle = displayDim / 2;
		cAABB aabb{ {middle.x - 350, middle.y - 350 }, {middle.x + 350, middle.y + 350} };
		//cAABB exaabb = aabb;
		//exaabb.min = aabb.min - (aabb.getExtents());
		//exaabb.max = aabb.max + (aabb.getExtents());
		cVec2 center = aabb.getCenter();
		CP_Settings_StrokeWeight(2);
		CP_Settings_Stroke(CP_Color_Create(200, 100, 0, 255));
		CP_Graphics_DrawLine(aabb.min.x, aabb.min.y, aabb.min.x, aabb.max.y);
		CP_Graphics_DrawLine(aabb.min.x, aabb.max.y, aabb.max.x, aabb.max.y);
		CP_Graphics_DrawLine(aabb.max.x, aabb.max.y, aabb.max.x, aabb.min.y);
		CP_Graphics_DrawLine(aabb.max.x, aabb.min.y, aabb.min.x, aabb.min.y);
		CP_Graphics_DrawCircle(center.x, center.y, 3);
		//CP_Settings_Stroke(CP_Color_Create(200, 100, 0, 255));
		//CP_Graphics_DrawLine(exaabb.min.x, exaabb.min.y, exaabb.min.x, exaabb.max.y);
		//CP_Graphics_DrawLine(exaabb.min.x, exaabb.max.y, exaabb.max.x, exaabb.max.y);
		//CP_Graphics_DrawLine(exaabb.max.x, exaabb.max.y, exaabb.max.x, exaabb.min.y);
		//CP_Graphics_DrawLine(exaabb.max.x, exaabb.min.y, exaabb.min.x, exaabb.min.y);
	}



	cVec2 mousePos = { CP_Input_GetMouseX(), CP_Input_GetMouseY() };
	if (CP_Input_MouseDown(MOUSE_BUTTON_2))
	{
		if (selectedPoint == cVec2::zero)
		{
			for (cVec2& pt : vdiagram.v_points)
			{
				if (distanceSqr(pt, mousePos) < 25.0f)
				{
					selectedPoint = pt;
					originalPoint = pt;
					break;
				}
			}
			return;
		}


		selectedPoint.x = mousePos.x;
		selectedPoint.y = mousePos.y;

		if (distanceSqr(selectedPoint, originalPoint) <= 4.0f)
			return; // hasn't move far enough to care about changing;

		timer -= dt;
		if (timer <= 0.0f)
		{
			timer = baseTimer;
			vdiagram.remove(originalPoint, false);
			vdiagram.add(selectedPoint);
			originalPoint = selectedPoint;
		}
	}
	else if (CP_Input_MouseReleased(MOUSE_BUTTON_2))
	{
		if (selectedPoint == cVec2::zero)
			return;

		vdiagram.remove(originalPoint, false);
		vdiagram.add(selectedPoint);
		originalPoint = cVec2::zero;
		selectedPoint = cVec2::zero;
		return;
	}

	if (CP_Input_KeyTriggered(KEY_A))
	{
		vdiagram.add(mousePos);
	}
	else if (CP_Input_KeyTriggered(KEY_D))
	{
		if (vdiagram.v_points.size() <= 3)
		{
			std::cout << "Voronoi Diagram MUST have at least 3 points!";
			return; // minimum viable diagram
		}

		std::vector<cVec2>& points = vdiagram.v_points;
		auto itr = std::find_if(points.begin(), points.end(), [&](const cVec2& p) {
			return distanceSqr(p, mousePos) <= 5.0f;
			});
		if (itr == points.end())
			return; // point not in diagram, ignore
		cVec2 fpoint = *itr;
		vdiagram.remove(fpoint);
	}
}

void SceneManager::HandleCameraInput()
{
	float camZoom = scenes[currentScene]->currentZoom;
	if (CP_Input_KeyDown(KEY_LEFT_BRACKET))
	{
		camZoom = c_clamp(camZoom - 0.5f, 1.0f, 1000.0f);
		drawer->ChangeZoom(camZoom);
		scenes[currentScene]->currentZoom = camZoom;
	}
	else if (CP_Input_KeyDown(KEY_RIGHT_BRACKET))
	{
		camZoom = c_clamp(camZoom + 0.5f, 1.0f, 1000.0f);
		drawer->ChangeZoom(camZoom);
		scenes[currentScene]->currentZoom = camZoom;
	}

	cVec2 moveDelta = cVec2::zero;
	bool moved = false;
	if (CP_Input_KeyDown(KEY_DOWN))
	{
		moveDelta.y -= 5.0f;
		moved = true;
	}
	else if (CP_Input_KeyDown(KEY_UP))
	{
		moveDelta.y += 5.0f;
		moved = true;
	}

	if (CP_Input_KeyDown(KEY_LEFT))
	{
		moveDelta.x -= 5.0f;
		moved = true;
	}
	else if (CP_Input_KeyDown(KEY_RIGHT))
	{
		moveDelta.x += 5.0f;
		moved = true;
	}

	if (moved)
	{
		drawer->PanCamera(moveDelta);
	}
}

void SceneManager::AddScene(PhysicsScene* scene)
{
	sceneCount++;
	scenes.push_back(scene);
}

void SceneManager::RemoveScene(int sceneIndex)
{
	if (sceneIndex >= 0 && sceneIndex < sceneCount)
	{
		delete scenes[sceneIndex];
		scenes.erase(scenes.begin() + sceneIndex);
		sceneCount--;
	}
}

PhysicsScene* SceneManager::GetCurrentScene()
{
	return scenes[currentScene];
}

void SceneManager::LoadPhysicsUI()
{
	std::vector<UIEventTrigger> events;
	UIEventTrigger drawAABBtrigger;
	drawAABBtrigger.type = UIEventType::OnMouseTriggered;
	drawAABBtrigger.mouse = MOUSE_BUTTON_1;
	drawAABBtrigger.callback = [&]() {
		printf("Toggling AABB display\n");
		drawer->draw.drawAABBs = !drawer->draw.drawAABBs;
		};
	events.push_back(drawAABBtrigger);

	UIEventTrigger drawTreeAABBtrigger;
	drawTreeAABBtrigger.type = UIEventType::OnMouseTriggered;
	drawTreeAABBtrigger.mouse = MOUSE_BUTTON_1;
	drawTreeAABBtrigger.callback = [&]() {
		printf("Toggling Tree AABB display\n");
		drawer->draw.drawTreeAABBs = !drawer->draw.drawTreeAABBs;
		};
	events.push_back(drawTreeAABBtrigger);

	UIEventTrigger drawMassTrigger;
	drawMassTrigger.type = UIEventType::OnMouseTriggered;
	drawMassTrigger.mouse = MOUSE_BUTTON_1;
	drawMassTrigger.callback = [&]() {
		printf("Toggling transform/mass display\n");
		drawer->draw.drawMass = !drawer->draw.drawMass;
		};
	events.push_back(drawMassTrigger);

	UIEventTrigger drawContactPointsTrigger;
	drawContactPointsTrigger.type = UIEventType::OnMouseTriggered;
	drawContactPointsTrigger.mouse = MOUSE_BUTTON_1;
	drawContactPointsTrigger.callback = [&]() {
		printf("Toggling contact point display\n");
		drawer->draw.drawContactPoints = !drawer->draw.drawContactPoints;
		};
	events.push_back(drawContactPointsTrigger);

	UIEventTrigger drawContactNormalsTrigger;
	drawContactNormalsTrigger.type = UIEventType::OnMouseTriggered;
	drawContactNormalsTrigger.mouse = MOUSE_BUTTON_1;
	drawContactNormalsTrigger.callback = [&]() {
		printf("Toggling contact normal display\n");
		drawer->draw.drawContactNormals = !drawer->draw.drawContactNormals;
		};
	events.push_back(drawContactNormalsTrigger);

	UIEventTrigger drawContactImpulsesTrigger;
	drawContactImpulsesTrigger.type = UIEventType::OnMouseTriggered;
	drawContactImpulsesTrigger.mouse = MOUSE_BUTTON_1;
	drawContactImpulsesTrigger.callback = [&]() {
		printf("Toggling contact impulse display\n");
		drawer->draw.drawContactImpulses = drawer->draw.drawContactImpulses;
		};
	events.push_back(drawContactImpulsesTrigger);

	UIEventTrigger drawFrictionImpulsesTrigger;
	drawFrictionImpulsesTrigger.type = UIEventType::OnMouseTriggered;
	drawFrictionImpulsesTrigger.mouse = MOUSE_BUTTON_1;
	drawFrictionImpulsesTrigger.callback = [&]() {
		printf("Toggling friction impulse display\n");
		drawer->draw.drawFrictionImpulses = !drawer->draw.drawFrictionImpulses;
		};
	events.push_back(drawFrictionImpulsesTrigger);

	std::vector<std::string> buttonNames = {
		"Toggle AABBs",
		"Toggle Tree AABBs",
		"Toggle Mass",
		"Toggle Contact Points",
		"Toggle Contact Normals",
		"Toggle Contact Impulses",
		"Toggle Friction Impulses"
	};

	std::vector<cVec2> nameOffsets =
	{
		{0.0f, 20.0f},
		{0.0f, 20.0f},
		{0.0f, 20.0f},
		{0.0f, 20.0f},
		{0.0f, 20.0f},
		{0.0f, 20.0f},
		{0.0f, 20.0f}
	};

	float spacing = 20.0f;
	float startX = 75.0f; // Initial horizontal offset
	float y = 40.0f;        // Fixed Y position near the top of the screen

	std::vector<cVec2> btnDims =
	{
		{ 80.0f, 30.0f },
		{ 110.0f, 30.0f },
		{ 70.0f, 30.0f },
		{ 125.0f, 30.0f },
		{ 140.0f, 30.0f },
		{ 140.0f, 30.0f },
		{ 150.0f, 30.0f },
	};

	float currentX = startX;
	for (int i = 0; i < buttonNames.size(); ++i)
	{
		float halfWidth = btnDims[i].x * 0.5f; // Half of current button width
		float prevHalfWidth = (i == 0) ? 0 : btnDims[i - 1].x * 0.5f; // Half of previous button width
		// Adjust X position to ensure exact spacing between button centers
		if (i > 0)
			currentX += prevHalfWidth + spacing + halfWidth;


		// Define button configuration
		UIComponentConfig buttonConfig{
			currentX - halfWidth, y,    // x, y position
			btnDims[i].x, btnDims[i].y,         // width, height
			0.9f, 0.9f, 0.9f, 1.0f,     // RGBA color
			buttonNames[i],      // Button text
			nameOffsets[i].x, nameOffsets[i].y,                // Text offset
			0.1f, 0.1f, 0.1f, 1.0f,     // Text color
			15.0f                       // Text size
		};
		uimanager->AddRectUIButton(buttonConfig, { events[i] });
	}
}
void SceneManager::LoadVoronoiUI()
{
	std::vector<UIEventTrigger> events;
	UIEventTrigger saveDiagram;
	saveDiagram.type = UIEventType::OnMouseTriggered;
	saveDiagram.mouse = MOUSE_BUTTON_1;
	saveDiagram.callback = [&]() {
		printf("Saving Diagram...\n");
		cVoronoiDiagram* vd = static_cast<cVoronoiDiagram*>(voronoi);
		cVec2 middle = drawer->getScreenDimensions() / 2;
		cFracturePattern savedPattern;
		savedPattern.pattern = *vd;
		savedPattern.pattern.transform(-middle, cRot::iden);
		savedPattern.min_extent = {middle.x - 350, middle.y - 350 };
		savedPattern.max_extent = {middle.x + 350, middle.y + 350 };
		VoronoiParser::saveDiagram(savedPattern);
		};
	events.push_back(saveDiagram);

	UIEventTrigger loadDiagram;
	loadDiagram.type = UIEventType::OnMouseTriggered;
	loadDiagram.mouse = MOUSE_BUTTON_1;
	loadDiagram.callback = [&]() {
		printf("Loading Diagram\n");
		cFracturePattern pattern = VoronoiParser::loadDiagram();
		cVoronoiDiagram* rvd = static_cast<cVoronoiDiagram*>(voronoi);
		*rvd = pattern.pattern;
		cVec2 displayMid = drawer->getScreenDimensions() / 2;
		(*rvd).transform(displayMid, cRot::iden);
		};
	events.push_back(loadDiagram);

	UIEventTrigger clearDiagram;
	clearDiagram.type = UIEventType::OnMouseTriggered;
	clearDiagram.mouse = MOUSE_BUTTON_1;
	clearDiagram.callback = [&]() {
		printf("Cleared Diagram!\n");
		cVoronoiDiagram* vd = static_cast<cVoronoiDiagram*>(voronoi);
		vd->clear();
		};
	events.push_back(clearDiagram);

	UIEventTrigger generateDiagram;
	generateDiagram.type = UIEventType::OnMouseTriggered;
	generateDiagram.mouse = MOUSE_BUTTON_1;
	generateDiagram.callback = [&]() {
		printf("Generating Diagram...\n");
		cVoronoiDiagram* vd = static_cast<cVoronoiDiagram*>(voronoi);
		vd->clear();
		int bufferEdgeWidth = 350;
		int numPoints = CP_Random_RangeInt(3, 33);
		std::vector<cVec2> points(numPoints);
		for (int i = 0; i < numPoints; i++) {
			float x = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowWidth() - bufferEdgeWidth);
			float y = CP_Random_RangeInt(bufferEdgeWidth, CP_System_GetWindowHeight() - bufferEdgeWidth / 2);
			points.push_back({ x,y });
		}
		vd->create(points.data(), points.size());
		};
	events.push_back(generateDiagram);

	std::vector<std::string> buttonNames = {
	  "Save",
	  "Load",
	  "Clear",
	  "Generate"
	};

	std::vector<cVec2> nameOffsets =
	{
		{10.0f, 20.0f},
		{10.0f, 20.0f},
		{10.0f, 20.0f},
		{4.0f, 20.0f},
	};

	float spacing = 20.0f;
	float startX = 75.0f; // Initial horizontal offset
	float y = 40.0f;        // Fixed Y position near the top of the screen

	std::vector<cVec2> btnDims =
	{
		{ 50.0f, 30.0f },
		{ 50.0f, 30.0f },
		{ 50.0f, 30.0f },
		{ 60.0f, 30.0f },
	};

	float currentX = startX;
	for (int i = 0; i < buttonNames.size(); ++i)
	{
		float halfWidth = btnDims[i].x * 0.5f; // Half of current button width
		float prevHalfWidth = (i == 0) ? 0 : btnDims[i - 1].x * 0.5f; // Half of previous button width
		// Adjust X position to ensure exact spacing between button centers
		if (i > 0)
			currentX += prevHalfWidth + spacing + halfWidth;


		// Define button configuration
		UIComponentConfig buttonConfig{
			currentX - halfWidth, y,    // x, y position
			btnDims[i].x, btnDims[i].y,         // width, height
			0.9f, 0.9f, 0.9f, 1.0f,     // RGBA color
			buttonNames[i],      // Button text
			nameOffsets[i].x, nameOffsets[i].y,                // Text offset
			0.1f, 0.1f, 0.1f, 1.0f,     // Text color
			15.0f                       // Text size
		};
		uimanager->AddRectUIButton(buttonConfig, { events[i] });
	}
}
#pragma endregion