#include <PxPhysicsAPI.h>
#include <iostream>
#include <vector>
#include <windows.h>
#include "dx12/DX12Renderer.h"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "simulation/SimulationRecorder.h"
#include "simulation/PerformanceProfiler.h"
#include "simulation/ExperimentController.h"
#include "simulation/DeformableVolumeManager.h"
#include "CommandLineArgs.h"
#include <filesystem>
#include <ctime>

// Forward declare Win32 message handler for ImGui
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

using namespace physx;

// Global PhysX variables
PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;
PxFoundation*			gFoundation = nullptr;
PxPhysics*				gPhysics = nullptr;
PxDefaultCpuDispatcher*	gDispatcher = nullptr;
PxScene*				gScene = nullptr;
PxMaterial*				gMaterial = nullptr;
PxCudaContextManager*	gCudaContextManager = nullptr;

// DirectX 12 Renderer
std::unique_ptr<DX12Renderer> gRenderer;

// Window variables
HWND gHwnd = nullptr;
constexpr int WINDOW_WIDTH = RenderConstants::kDefaultWindowWidth;
constexpr int WINDOW_HEIGHT = RenderConstants::kDefaultWindowHeight;
bool gRunning = true;

// Mouse tracking
bool gMouseCapturedOrbit = false;  // Right button for orbit
bool gMouseCapturedPan = false;     // Middle button for pan
POINT gLastMousePos;

// Simulation research systems
std::unique_ptr<SimulationRecorder> gRecorder;
std::unique_ptr<PerformanceProfiler> gProfiler;
std::unique_ptr<ExperimentController> gController;

// Command line arguments for research experiments
CommandLineArgs gArgs;

// Continuous spawning
bool gEnableContinuousSpawn = false;
float gLastSpawnTime = 0.0f;
float gSpawnInterval = 1.0f;  // Spawn every 1 second

// Scene selection
enum class SceneType
{
    BASIC_STACK,
    SHAPES_VARIETY,
    FALLING_OBJECTS,
    PYRAMID,
    STRESS_TEST,
    SNIPPET_HELLO_WORLD,
    PENDULUM,
    CHAIN,
    MECHANISM,
    BREAKABLE_WALL,
    ROBOT_ARM,
    CONVEX_SHAPES,
    TERRAIN,
    SOFT_BODY
};
SceneType gCurrentSceneType = SceneType::BASIC_STACK;

// Store articulations for cleanup
std::vector<PxArticulationReducedCoordinate*> gArticulations;

// Deformable volume (soft body) manager
std::unique_ptr<DeformableVolumeManager> gDeformableManager;

// Store joints for cleanup
std::vector<PxJoint*> gJoints;

// Forward declarations
bool initPhysics();
void stepPhysics(float deltaTime);
void cleanupPhysics();
PxRigidDynamic* createBox(const PxVec3& position, const PxVec3& halfExtents, float density = 1.0f);
PxRigidDynamic* createSphere(const PxVec3& position, float radius, float density = 1.0f);
PxRigidStatic* createGround();
void createSceneBasicStack();
void createSceneShapesVariety();
void createSceneFallingObjects();
void createScenePyramid();
void createSceneStressTest();
void createSceneSnippetHelloWorld();
void createScenePendulum();
void createSceneChain();
void createSceneMechanism();
void createSceneBreakableWall();
void createSceneRobotArm();
void createSceneConvexShapes();
void createSceneTerrain();
void createSceneSoftBody();
void createInitialScene();
void resetScene();
void switchScene(SceneType newType);

// Collision callback class
class CollisionCallback : public PxSimulationEventCallback
{
public:
    void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override
    {
        for (PxU32 i = 0; i < count; i++)
        {
            PxJoint* joint = reinterpret_cast<PxJoint*>(constraints[i].externalReference);
            if (joint)
            {
                // Get actors connected by this joint
                PxRigidActor* actor0 = nullptr;
                PxRigidActor* actor1 = nullptr;
                joint->getActors(actor0, actor1);

                // Calculate break position (midpoint between actors)
                PxVec3 breakPos(0, 0, 0);
                if (actor0)
                {
                    breakPos = actor0->getGlobalPose().p;
                }
                if (actor1)
                {
                    breakPos = (breakPos + actor1->getGlobalPose().p) * 0.5f;
                }

                // Add visual break effect
                if (gRenderer)
                {
                    gRenderer->AddBreakEffect(breakPos);
                }

                std::cout << "Joint broken at position (" << breakPos.x << ", "
                          << breakPos.y << ", " << breakPos.z << ")" << std::endl;
            }
        }
    }
    void onWake(PxActor** actors, PxU32 count) override {}
    void onSleep(PxActor** actors, PxU32 count) override {}
    void onTrigger(PxTriggerPair* pairs, PxU32 count) override {}
    void onAdvance(const PxRigidBody* const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count) override {}

    void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override
    {
        for (PxU32 i = 0; i < nbPairs; i++)
        {
            const PxContactPair& cp = pairs[i];

            // Only process if we have contact points
            if (cp.contactCount > 0)
            {
                // Extract contact points
                PxContactPairPoint contactPoints[16];
                PxU32 nbContacts = cp.extractContacts(contactPoints, 16);

                for (PxU32 j = 0; j < nbContacts; j++)
                {
                    const PxContactPairPoint& contact = contactPoints[j];

                    // Calculate impulse magnitude
                    float impulse = contact.impulse.magnitude();

                    // Add collision point to renderer
                    if (gRenderer)
                    {
                        gRenderer->AddCollisionPoint(contact.position, impulse);

                        // Add contact force for force visualization (with normal for normal visualization)
                        PxVec3 force = contact.normal * impulse;
                        gRenderer->AddContactForceWithNormal(contact.position, force, contact.normal);
                    }

                    // Record collision in simulation recorder if active
                    if (gRecorder && gRecorder->IsRecording())
                    {
                        gRecorder->RecordCollision(
                            pairHeader.actors[0],
                            pairHeader.actors[1],
                            contact.position,
                            impulse
                        );
                    }
                }
            }
        }
    }
};

CollisionCallback gCollisionCallback;

// Window procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    // Check if Shift key is held - allows camera control even over ImGui windows
    bool shiftHeld = (GetKeyState(VK_SHIFT) & 0x8000) != 0;

    // Check if ImGui is initialized before accessing its state
    bool imguiInitialized = (ImGui::GetCurrentContext() != nullptr);

    // Let ImGui handle the message first (unless Shift is held for camera override)
    if (imguiInitialized && !shiftHeld && ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
        return true;

    // Check if we should allow camera input (either ImGui doesn't want it, or Shift is held, or ImGui not initialized)
    bool allowCameraInput = !imguiInitialized || shiftHeld || !ImGui::GetIO().WantCaptureMouse;
    bool allowKeyboardInput = !imguiInitialized || shiftHeld || !ImGui::GetIO().WantCaptureKeyboard;

    switch (uMsg)
    {
    case WM_DESTROY:
        gRunning = false;
        PostQuitMessage(0);
        return 0;

    case WM_KEYDOWN:
        // Handle camera input (works when: ImGui doesn't want keyboard OR Shift is held)
        if (gRenderer && allowKeyboardInput)
        {
            gRenderer->GetCamera().HandleInput(wParam);
        }

        if (wParam == VK_ESCAPE)
        {
            gRunning = false;
            PostQuitMessage(0);
        }
        return 0;

    case WM_KEYUP:
        // Handle key release for smooth camera movement
        if (gRenderer && allowKeyboardInput)
        {
            gRenderer->GetCamera().HandleKeyUp(wParam);
        }
        return 0;

    case WM_RBUTTONDOWN:
        // Right button: Orbit camera (works when: ImGui doesn't want mouse OR Shift is held)
        if (allowCameraInput)
        {
            gMouseCapturedOrbit = true;
            GetCursorPos(&gLastMousePos);
            ScreenToClient(hwnd, &gLastMousePos);
            SetCapture(hwnd);
            ShowCursor(FALSE);
        }
        return 0;

    case WM_RBUTTONUP:
        if (gMouseCapturedOrbit)
        {
            gMouseCapturedOrbit = false;
            ReleaseCapture();
            ShowCursor(TRUE);
        }
        return 0;

    case WM_MBUTTONDOWN:
        // Middle button: Pan camera (works when: ImGui doesn't want mouse OR Shift is held)
        if (allowCameraInput)
        {
            gMouseCapturedPan = true;
            GetCursorPos(&gLastMousePos);
            ScreenToClient(hwnd, &gLastMousePos);
            SetCapture(hwnd);
        }
        return 0;

    case WM_MBUTTONUP:
        if (gMouseCapturedPan)
        {
            gMouseCapturedPan = false;
            ReleaseCapture();
        }
        return 0;

    case WM_MOUSEMOVE:
        // Mouse move for camera: works when mouse is captured OR (camera input allowed)
        if (gRenderer && (gMouseCapturedOrbit || gMouseCapturedPan || allowCameraInput))
        {
            POINT currentPos;
            GetCursorPos(&currentPos);
            ScreenToClient(hwnd, &currentPos);

            int dx = currentPos.x - gLastMousePos.x;
            int dy = currentPos.y - gLastMousePos.y;

            if (gMouseCapturedOrbit)
            {
                gRenderer->GetCamera().HandleMouseOrbit(dx, dy);
            }
            else if (gMouseCapturedPan)
            {
                gRenderer->GetCamera().HandleMousePan(dx, dy);
            }

            gLastMousePos = currentPos;
        }
        return 0;

    case WM_MOUSEWHEEL:
        // Mouse wheel: Zoom camera (works when: ImGui doesn't want mouse OR Shift is held)
        if (gRenderer && allowCameraInput)
        {
            int delta = GET_WHEEL_DELTA_WPARAM(wParam);
            gRenderer->GetCamera().HandleMouseZoom(delta / 120.0f);  // Normalize wheel delta
        }
        return 0;

    case WM_SIZE:
        if (gRenderer && wParam != SIZE_MINIMIZED)
        {
            UINT width = LOWORD(lParam);
            UINT height = HIWORD(lParam);
            gRenderer->Resize(width, height);
        }
        return 0;
    }

    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

// Create window
bool CreateAppWindow(HINSTANCE hInstance)
{
    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(WNDCLASSEXW);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = L"PhysXDX12Window";

    RegisterClassExW(&wc);

    RECT windowRect = { 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT };
    AdjustWindowRect(&windowRect, WS_OVERLAPPEDWINDOW, FALSE);

    gHwnd = CreateWindowExW(
        0,
        L"PhysXDX12Window",
        L"PhysX with DirectX 12 Visualization",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        windowRect.right - windowRect.left,
        windowRect.bottom - windowRect.top,
        nullptr,
        nullptr,
        hInstance,
        nullptr
    );

    if (!gHwnd)
    {
        return false;
    }

    ShowWindow(gHwnd, SW_SHOW);
    UpdateWindow(gHwnd);

    return true;
}

// Initialize PhysX
bool initPhysics()
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    if (!gFoundation)
    {
        std::cerr << "PxCreateFoundation failed!" << std::endl;
        return false;
    }

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, nullptr);
    if (!gPhysics)
    {
        std::cerr << "PxCreatePhysics failed!" << std::endl;
        return false;
    }

    // Initialize PhysX extensions (required for joints)
    if (!PxInitExtensions(*gPhysics, nullptr))
    {
        std::cerr << "PxInitExtensions failed!" << std::endl;
        return false;
    }
    std::cout << "PhysX Extensions initialized (joints enabled)" << std::endl;

    // Try to create CUDA context manager for GPU acceleration
    PxCudaContextManagerDesc cudaContextManagerDesc;
    gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc);
    if (gCudaContextManager)
    {
        if (!gCudaContextManager->contextIsValid())
        {
            std::cout << "WARNING: CUDA Context Manager created but context is INVALID" << std::endl;
            gCudaContextManager->release();
            gCudaContextManager = nullptr;
        }
        else
        {
            std::cout << "SUCCESS: GPU PhysX (CUDA) is ENABLED!" << std::endl;
        }
    }
    else
    {
        std::cout << "WARNING: GPU PhysX (CUDA) is NOT AVAILABLE - using CPU only" << std::endl;
    }

    // Scene setup
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;

    // Enable GPU dynamics if CUDA is available
    if (gCudaContextManager)
    {
        sceneDesc.cudaContextManager = gCudaContextManager;
        sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
        sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
        std::cout << "  Scene configured for GPU dynamics" << std::endl;
    }

    // Set collision callback
    sceneDesc.simulationEventCallback = &gCollisionCallback;

    gScene = gPhysics->createScene(sceneDesc);

    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    // Initialize deformable volume (soft body) manager
    gDeformableManager = std::make_unique<DeformableVolumeManager>();
    if (gDeformableManager->Initialize(gPhysics, gCudaContextManager))
    {
        std::cout << "Soft body simulation enabled (GPU required)" << std::endl;
    }
    else
    {
        std::cout << "Soft body simulation NOT available (GPU/CUDA required)" << std::endl;
    }

    std::cout << "PhysX initialized successfully!" << std::endl;
    return true;
}

// From SnippetHelloWorld - Create a stack of boxes
static PxReal stackZ = 10.0f;

static void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
    PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
    for(PxU32 i=0; i<size; i++)
    {
        for(PxU32 j=0; j<size-i; j++)
        {
            PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
            PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
            body->attachShape(*shape);
            PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
            gScene->addActor(*body);
        }
    }
    shape->release();
}

static PxRigidDynamic* createDynamicSphere(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
    dynamic->setAngularDamping(0.5f);
    dynamic->setLinearVelocity(velocity);
    gScene->addActor(*dynamic);
    return dynamic;
}

// ========== Test Scene 1: Snippet HelloWorld ==========
void createSceneSnippetHelloWorld()
{
    std::cout << "Creating SnippetHelloWorld scene..." << std::endl;

    // Create static ground plane - exactly as in SnippetHelloWorld
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    // Create visible ground box for rendering
    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(100.0f, 0.5f, 100.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Reset stackZ for this scene
    stackZ = 10.0f;

    // Create 5 stacks of boxes - exactly as in SnippetHelloWorld
    for(PxU32 i=0; i<5; i++)
    {
        createStack(PxTransform(PxVec3(0, 0, stackZ-=10.0f)), 10, 2.0f);
    }

    // Create a sphere that will knock down the stacks
    createDynamicSphere(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));

    std::cout << "SnippetHelloWorld scene created - Total actors: "
              << gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC) << std::endl;
}

// ========== Test Scene 2: Basic Stack ==========
void createSceneBasicStack()
{
    std::cout << "Creating Basic Stack scene..." << std::endl;

    // Ground plane
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);
    std::cout << "  Created ground plane" << std::endl;

    // Visible ground
    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();
    std::cout << "  Created visible ground box at (0, -0.5, 0)" << std::endl;

    // Create a simple stack of 10 boxes
    float boxSize = 2.0f;
    std::cout << "  Creating stack of 10 boxes (boxSize=" << boxSize << ")..." << std::endl;

    for (int i = 0; i < 10; i++)
    {
        PxVec3 position(0, boxSize * 0.5f + boxSize * i, 0);
        std::cout << "    Box " << i << ": Creating at position ("
                  << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;

        PxBoxGeometry boxGeom(boxSize * 0.5f, boxSize * 0.5f, boxSize * 0.5f);
        PxRigidDynamic* box = PxCreateDynamic(*gPhysics, PxTransform(position), boxGeom, *gMaterial, 10.0f);

        if (box)
        {
            box->setAngularDamping(0.5f);
            gScene->addActor(*box);

            // Verify the box was added and check its actual position
            PxTransform actualTransform = box->getGlobalPose();
            std::cout << "    Box " << i << ": ADDED to scene. Actual position: ("
                      << actualTransform.p.x << ", " << actualTransform.p.y << ", " << actualTransform.p.z << ")" << std::endl;
        }
        else
        {
            std::cout << "    Box " << i << ": ERROR - PxCreateDynamic returned NULL!" << std::endl;
        }
    }

    // Final verification
    PxU32 nbActorsAfter = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    PxU32 nbDynamic = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    std::cout << "  Basic Stack scene created - Total actors: " << nbActorsAfter
              << " (Dynamic: " << nbDynamic << ")" << std::endl;
}

// ========== Test Scene 3: Shapes Variety ==========
void createSceneShapesVariety()
{
    std::cout << "Creating Shapes Variety scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Row of boxes
    for (int i = 0; i < 5; i++)
    {
        PxVec3 position(-10.0f + i * 5.0f, 5.0f, 0);
        createBox(position, PxVec3(1.0f, 1.0f, 1.0f), 10.0f);
    }

    // Row of spheres
    for (int i = 0; i < 5; i++)
    {
        PxVec3 position(-10.0f + i * 5.0f, 15.0f, 0);
        createSphere(position, 1.0f, 10.0f);
    }

    // Mixed stack in center
    createBox(PxVec3(0, 2, 0), PxVec3(2, 0.5f, 2), 10.0f);
    createSphere(PxVec3(0, 5, 0), 1.0f, 10.0f);
    createBox(PxVec3(0, 8, 0), PxVec3(1.5f, 1.5f, 1.5f), 10.0f);
    createSphere(PxVec3(0, 12, 0), 1.2f, 10.0f);

    std::cout << "Shapes Variety scene created" << std::endl;
}

// ========== Test Scene 4: Falling Objects ==========
void createSceneFallingObjects()
{
    std::cout << "Creating Falling Objects scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create random falling objects
    srand(42);  // Fixed seed for reproducibility
    for (int i = 0; i < 30; i++)
    {
        float x = (rand() % 40 - 20) / 1.0f;
        float y = 20.0f + (rand() % 30);
        float z = (rand() % 40 - 20) / 1.0f;

        if (rand() % 2 == 0)
        {
            // Box
            float size = 0.5f + (rand() % 10) / 10.0f;
            createBox(PxVec3(x, y, z), PxVec3(size, size, size), 10.0f);
        }
        else
        {
            // Sphere
            float radius = 0.5f + (rand() % 10) / 10.0f;
            createSphere(PxVec3(x, y, z), radius, 10.0f);
        }
    }

    std::cout << "Falling Objects scene created" << std::endl;
}

// ========== Test Scene 5: Pyramid ==========
void createScenePyramid()
{
    std::cout << "Creating Pyramid scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create pyramid
    float boxSize = 2.0f;
    int pyramidSize = 8;

    for (int layer = 0; layer < pyramidSize; layer++)
    {
        int boxesInLayer = pyramidSize - layer;
        float layerOffset = -boxesInLayer * boxSize * 0.5f;

        for (int i = 0; i < boxesInLayer; i++)
        {
            for (int j = 0; j < boxesInLayer; j++)
            {
                PxVec3 position(
                    layerOffset + i * boxSize + boxSize * 0.5f,
                    boxSize * 0.5f + layer * boxSize,
                    layerOffset + j * boxSize + boxSize * 0.5f
                );
                createBox(position, PxVec3(boxSize * 0.5f, boxSize * 0.5f, boxSize * 0.5f), 10.0f);
            }
        }
    }

    // Add a sphere to knock it down
    createDynamicSphere(PxTransform(PxVec3(25, 10, 0)), PxSphereGeometry(3), PxVec3(-50, 0, 0));

    std::cout << "Pyramid scene created" << std::endl;
}

// ========== Test Scene 6: Stress Test ==========
void createSceneStressTest()
{
    std::cout << "Creating Stress Test scene (large number of objects)..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(100.0f, 0.5f, 100.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create grid of boxes
    int gridSize = 20;
    float spacing = 3.0f;
    float startX = -gridSize * spacing * 0.5f;
    float startZ = -gridSize * spacing * 0.5f;

    srand(123);
    for (int x = 0; x < gridSize; x++)
    {
        for (int z = 0; z < gridSize; z++)
        {
            float posX = startX + x * spacing;
            float posZ = startZ + z * spacing;
            float height = 2.0f + (rand() % 20);

            PxVec3 position(posX, height, posZ);
            if (rand() % 3 == 0)
            {
                createSphere(position, 0.5f, 10.0f);
            }
            else
            {
                createBox(position, PxVec3(0.5f, 0.5f, 0.5f), 10.0f);
            }
        }
    }

    std::cout << "Stress Test scene created with " << (gridSize * gridSize) << " objects" << std::endl;
}

// ========== Test Scene 7: Pendulum ==========
void createScenePendulum()
{
    std::cout << "Creating Pendulum scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create a static anchor point
    PxRigidStatic* anchor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 15.0f, 0)));
    PxBoxGeometry anchorGeom(0.5f, 0.5f, 0.5f);
    PxShape* anchorShape = PxRigidActorExt::createExclusiveShape(*anchor, anchorGeom, *gMaterial);
    gScene->addActor(*anchor);

    // Create pendulum bob - start at an angle (45 degrees) so it swings
    float ropeLength = 8.0f;
    float startAngle = PxPi / 4.0f;  // 45 degrees
    float bobX = ropeLength * sinf(startAngle);
    float bobY = 15.0f - ropeLength * cosf(startAngle);
    PxRigidDynamic* bob = createSphere(PxVec3(bobX, bobY, 0), 1.0f, 10.0f);

    // Create distance joint (acts like a rope)
    PxDistanceJoint* joint = PxDistanceJointCreate(*gPhysics,
        anchor, PxTransform(PxVec3(0, 0, 0)),
        bob, PxTransform(PxVec3(0, 0, 0)));

    if (joint)
    {
        // Set the rope length
        joint->setMinDistance(ropeLength);
        joint->setMaxDistance(ropeLength);
        joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
        joint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
        joint->setStiffness(0.0f);
        joint->setDamping(0.0f);

        gJoints.push_back(joint);
        std::cout << "  Created distance joint for pendulum (rope length: " << ropeLength << ")" << std::endl;
    }

    std::cout << "Pendulum scene created!" << std::endl;
}

// ========== Test Scene 8: Chain ==========
void createSceneChain()
{
    std::cout << "Creating Chain scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Chain parameters - use distance joints for stability
    const int numLinks = 8;
    const float linkSpacing = 1.2f;
    const float linkRadius = 0.4f;

    // Create anchor point
    PxRigidStatic* anchor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 15.0f, 0)));
    PxBoxGeometry anchorGeom(0.5f, 0.5f, 0.5f);
    PxShape* anchorShape = PxRigidActorExt::createExclusiveShape(*anchor, anchorGeom, *gMaterial);
    gScene->addActor(*anchor);

    PxRigidActor* prevActor = anchor;

    // Create chain links using distance joints (more stable than spherical)
    for (int i = 0; i < numLinks; i++)
    {
        float y = 15.0f - (i + 1) * linkSpacing;
        PxRigidDynamic* link = createSphere(PxVec3(0, y, 0), linkRadius, 2.0f);

        // Increase solver iterations for stability
        link->setSolverIterationCounts(8, 4);

        // Create distance joint
        PxDistanceJoint* joint = PxDistanceJointCreate(*gPhysics,
            prevActor, PxTransform(PxVec3(0, 0, 0)),
            link, PxTransform(PxVec3(0, 0, 0)));

        if (joint)
        {
            joint->setMinDistance(linkSpacing * 0.9f);
            joint->setMaxDistance(linkSpacing);
            joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
            joint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
            joint->setStiffness(1000.0f);
            joint->setDamping(100.0f);
            gJoints.push_back(joint);
        }

        prevActor = link;
    }

    // Add a weight at the end
    float weightY = 15.0f - (numLinks + 1) * linkSpacing;
    PxRigidDynamic* weight = createBox(PxVec3(0, weightY, 0), PxVec3(0.8f, 0.8f, 0.8f), 10.0f);
    weight->setSolverIterationCounts(8, 4);

    PxDistanceJoint* finalJoint = PxDistanceJointCreate(*gPhysics,
        prevActor, PxTransform(PxVec3(0, 0, 0)),
        weight, PxTransform(PxVec3(0, 0.8f, 0)));

    if (finalJoint)
    {
        finalJoint->setMinDistance(linkSpacing * 0.9f);
        finalJoint->setMaxDistance(linkSpacing);
        finalJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
        finalJoint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
        finalJoint->setStiffness(1000.0f);
        finalJoint->setDamping(100.0f);
        gJoints.push_back(finalJoint);
    }

    std::cout << "Chain scene created with " << numLinks << " links!" << std::endl;
}

// ========== Test Scene 9: Mechanism ==========
void createSceneMechanism()
{
    std::cout << "Creating Mechanism scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create a simple rotating arm with motor
    // Static base
    PxRigidStatic* base = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 5.0f, 0)));
    PxBoxGeometry baseGeom(0.6f, 0.6f, 0.6f);
    PxShape* baseShape = PxRigidActorExt::createExclusiveShape(*base, baseGeom, *gMaterial);
    gScene->addActor(*base);

    // Rotating arm
    float armLength = 4.0f;
    PxRigidDynamic* arm = createBox(PxVec3(armLength * 0.5f, 5.0f, 0),
                                     PxVec3(armLength * 0.5f, 0.3f, 0.3f), 5.0f);
    arm->setSolverIterationCounts(16, 8);

    // Revolute joint with motor - rotate around Z axis
    PxTransform baseFrame(PxVec3(0, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
    PxTransform armFrame(PxVec3(-armLength * 0.5f, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 1, 0)));

    PxRevoluteJoint* armJoint = PxRevoluteJointCreate(*gPhysics,
        base, baseFrame,
        arm, armFrame);

    if (armJoint)
    {
        armJoint->setDriveVelocity(1.5f);  // 1.5 rad/s (slow rotation)
        armJoint->setDriveForceLimit(100.0f);  // Limited force
        armJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
        gJoints.push_back(armJoint);
        std::cout << "  Created motorized rotating arm" << std::endl;
    }

    // Add a weight at the end of the arm
    PxRigidDynamic* weight = createSphere(PxVec3(armLength, 5.0f, 0), 0.5f, 3.0f);
    weight->setSolverIterationCounts(16, 8);

    // Fixed joint to attach weight to arm end
    PxFixedJoint* weightJoint = PxFixedJointCreate(*gPhysics,
        arm, PxTransform(PxVec3(armLength * 0.5f, 0, 0)),
        weight, PxTransform(PxVec3(0, 0, 0)));

    if (weightJoint)
    {
        gJoints.push_back(weightJoint);
    }

    // Create a second arm connected to the first
    float arm2Length = 3.0f;
    PxRigidDynamic* arm2 = createBox(PxVec3(armLength + arm2Length * 0.5f, 5.0f, 0),
                                      PxVec3(arm2Length * 0.5f, 0.2f, 0.2f), 2.0f);
    arm2->setSolverIterationCounts(16, 8);

    // Revolute joint connecting arm2 to weight
    PxRevoluteJoint* arm2Joint = PxRevoluteJointCreate(*gPhysics,
        weight, PxTransform(PxVec3(0, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 1, 0))),
        arm2, PxTransform(PxVec3(-arm2Length * 0.5f, 0, 0), PxQuat(PxHalfPi, PxVec3(0, 1, 0))));

    if (arm2Joint)
    {
        gJoints.push_back(arm2Joint);
        std::cout << "  Created second arm with revolute joint" << std::endl;
    }

    std::cout << "Mechanism scene created!" << std::endl;
}

// ========== Test Scene 10: Breakable Wall ==========
void createSceneBreakableWall()
{
    std::cout << "Creating Breakable Wall scene..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Wall parameters
    const int wallWidth = 8;
    const int wallHeight = 6;
    const float brickWidth = 1.0f;
    const float brickHeight = 0.5f;
    const float brickDepth = 0.4f;
    const float breakForce = 200.0f;  // Lower force so joints can break
    const float breakTorque = 200.0f;

    // Create wall of bricks connected by breakable fixed joints
    std::vector<std::vector<PxRigidDynamic*>> bricks(wallHeight, std::vector<PxRigidDynamic*>(wallWidth, nullptr));

    for (int row = 0; row < wallHeight; row++)
    {
        float rowOffset = (row % 2 == 0) ? 0.0f : brickWidth * 0.5f;
        float y = brickHeight * 0.5f + row * brickHeight;

        for (int col = 0; col < wallWidth; col++)
        {
            float x = col * brickWidth - (wallWidth * brickWidth * 0.5f) + brickWidth * 0.5f + rowOffset;

            PxRigidDynamic* brick = createBox(PxVec3(x, y, 0), PxVec3(brickWidth * 0.45f, brickHeight * 0.45f, brickDepth * 0.5f), 5.0f);
            bricks[row][col] = brick;

            // Connect to brick below
            if (row > 0)
            {
                // Find the brick(s) below to connect to
                int belowCol = col;
                if (row % 2 == 0)
                {
                    // Even row: connect to brick below and to the left (if exists)
                    if (belowCol >= 0 && belowCol < wallWidth && bricks[row-1][belowCol])
                    {
                        PxTransform frame1(PxVec3(0, brickHeight * 0.5f, 0));
                        PxTransform frame2(PxVec3(0, -brickHeight * 0.5f, 0));

                        PxFixedJoint* joint = PxFixedJointCreate(*gPhysics,
                            bricks[row-1][belowCol], frame1,
                            brick, frame2);

                        if (joint)
                        {
                            joint->setBreakForce(breakForce, breakTorque);
                            gJoints.push_back(joint);
                        }
                    }
                }
                else
                {
                    // Odd row: connect to brick below
                    if (belowCol >= 0 && belowCol < wallWidth && bricks[row-1][belowCol])
                    {
                        PxTransform frame1(PxVec3(brickWidth * 0.25f, brickHeight * 0.5f, 0));
                        PxTransform frame2(PxVec3(-brickWidth * 0.25f, -brickHeight * 0.5f, 0));

                        PxFixedJoint* joint = PxFixedJointCreate(*gPhysics,
                            bricks[row-1][belowCol], frame1,
                            brick, frame2);

                        if (joint)
                        {
                            joint->setBreakForce(breakForce, breakTorque);
                            gJoints.push_back(joint);
                        }
                    }
                }
            }

            // Connect to brick on the left
            if (col > 0 && bricks[row][col-1])
            {
                PxTransform frame1(PxVec3(brickWidth * 0.45f, 0, 0));
                PxTransform frame2(PxVec3(-brickWidth * 0.45f, 0, 0));

                PxFixedJoint* joint = PxFixedJointCreate(*gPhysics,
                    bricks[row][col-1], frame1,
                    brick, frame2);

                if (joint)
                {
                    joint->setBreakForce(breakForce, breakTorque);
                    gJoints.push_back(joint);
                }
            }
        }
    }

    // Create a heavy projectile (wrecking ball)
    PxRigidDynamic* projectile = createSphere(PxVec3(-15.0f, 2.0f, 0.0f), 2.0f, 200.0f);
    projectile->setLinearVelocity(PxVec3(40.0f, 8.0f, 0.0f));  // Launch at the wall with higher velocity

    std::cout << "Breakable Wall scene created with " << (wallWidth * wallHeight) << " bricks!" << std::endl;
    std::cout << "  Number of breakable joints: " << gJoints.size() << std::endl;
    std::cout << "  Projectile launched at wall!" << std::endl;
}

// ========== Test Scene 11: Robot Arm (Articulation) ==========
void createSceneRobotArm()
{
    std::cout << "Creating Robot Arm scene (using Articulation)..." << std::endl;

    // Ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create articulation
    PxArticulationReducedCoordinate* articulation = gPhysics->createArticulationReducedCoordinate();
    if (!articulation)
    {
        std::cerr << "Failed to create articulation!" << std::endl;
        return;
    }

    // Set articulation flags
    articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);  // Fixed base
    articulation->setSolverIterationCounts(32, 4);

    // Create base link (fixed to ground)
    PxTransform basePose(PxVec3(0, 1.0f, 0));
    PxArticulationLink* baseLink = articulation->createLink(nullptr, basePose);
    PxRigidActorExt::createExclusiveShape(*baseLink, PxBoxGeometry(0.6f, 0.6f, 0.6f), *gMaterial);
    PxRigidBodyExt::updateMassAndInertia(*baseLink, 20.0f);

    // Create first arm segment (shoulder)
    float segmentLength = 3.0f;
    PxTransform arm1Pose(PxVec3(segmentLength * 0.5f, 1.6f, 0));  // Start horizontal
    PxArticulationLink* arm1 = articulation->createLink(baseLink, arm1Pose);
    PxRigidActorExt::createExclusiveShape(*arm1, PxBoxGeometry(segmentLength * 0.5f, 0.3f, 0.3f), *gMaterial);
    PxRigidBodyExt::updateMassAndInertia(*arm1, 8.0f);

    // Configure first joint (revolute - shoulder up/down)
    PxArticulationJointReducedCoordinate* joint1 = arm1->getInboundJoint();
    if (joint1)
    {
        joint1->setJointType(PxArticulationJointType::eREVOLUTE);
        joint1->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);

        // Set local poses - joint at the connection point
        joint1->setParentPose(PxTransform(PxVec3(0.6f, 0.0f, 0)));
        joint1->setChildPose(PxTransform(PxVec3(-segmentLength * 0.5f, 0, 0)));

        // Strong drive to swing arm up and down
        joint1->setDriveParams(PxArticulationAxis::eSWING2, PxArticulationDrive(2000.0f, 200.0f, PX_MAX_F32));
        joint1->setDriveVelocity(PxArticulationAxis::eSWING2, 1.5f);  // Continuous swing
    }

    // Create second arm segment (elbow)
    float segment2Length = 2.5f;
    PxTransform arm2Pose(PxVec3(segmentLength + segment2Length * 0.5f, 1.6f, 0));
    PxArticulationLink* arm2 = articulation->createLink(arm1, arm2Pose);
    PxRigidActorExt::createExclusiveShape(*arm2, PxBoxGeometry(segment2Length * 0.5f, 0.25f, 0.25f), *gMaterial);
    PxRigidBodyExt::updateMassAndInertia(*arm2, 5.0f);

    // Configure second joint (elbow bend)
    PxArticulationJointReducedCoordinate* joint2 = arm2->getInboundJoint();
    if (joint2)
    {
        joint2->setJointType(PxArticulationJointType::eREVOLUTE);
        joint2->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eLIMITED);
        joint2->setLimitParams(PxArticulationAxis::eSWING2, PxArticulationLimit(-PxPi * 0.8f, PxPi * 0.1f));

        joint2->setParentPose(PxTransform(PxVec3(segmentLength * 0.5f, 0, 0)));
        joint2->setChildPose(PxTransform(PxVec3(-segment2Length * 0.5f, 0, 0)));

        // Drive elbow to bend
        joint2->setDriveParams(PxArticulationAxis::eSWING2, PxArticulationDrive(1500.0f, 150.0f, PX_MAX_F32));
        joint2->setDriveVelocity(PxArticulationAxis::eSWING2, -0.8f);
    }

    // Create third segment (hand/gripper)
    PxTransform arm3Pose(PxVec3(segmentLength + segment2Length + 0.4f, 1.6f, 0));
    PxArticulationLink* arm3 = articulation->createLink(arm2, arm3Pose);
    PxRigidActorExt::createExclusiveShape(*arm3, PxBoxGeometry(0.4f, 0.2f, 0.4f), *gMaterial);
    PxRigidBodyExt::updateMassAndInertia(*arm3, 2.0f);

    // Configure third joint (wrist rotation)
    PxArticulationJointReducedCoordinate* joint3 = arm3->getInboundJoint();
    if (joint3)
    {
        joint3->setJointType(PxArticulationJointType::eREVOLUTE);
        joint3->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);

        joint3->setParentPose(PxTransform(PxVec3(segment2Length * 0.5f, 0, 0)));
        joint3->setChildPose(PxTransform(PxVec3(-0.4f, 0, 0)));

        // Spin the wrist
        joint3->setDriveParams(PxArticulationAxis::eTWIST, PxArticulationDrive(500.0f, 50.0f, PX_MAX_F32));
        joint3->setDriveVelocity(PxArticulationAxis::eTWIST, 3.0f);  // Fast spin
    }

    // Add articulation to scene
    gScene->addArticulation(*articulation);
    gArticulations.push_back(articulation);

    // Wake up the articulation
    articulation->wakeUp();

    // Add some objects for the arm to knock over
    for (int i = 0; i < 8; i++)
    {
        float x = 4.0f + i * 0.8f;
        float y = 0.5f + (i % 3) * 0.5f;
        createBox(PxVec3(x, y, 0.0f), PxVec3(0.3f, 0.3f, 0.3f), 1.0f);
    }

    std::cout << "Robot Arm scene created with 3-joint articulated arm!" << std::endl;
}

// ========== Test Scene 12: Convex Shapes ==========
void createSceneConvexShapes()
{
    std::cout << "Creating Convex Shapes scene (using Mesh Cooking)..." << std::endl;

    // Ground plane
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    // Create a visible ground box
    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(30.0f, 0.5f, 30.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create a triangle mesh ramp (static only)
    {
        float rampScale = 5.0f;
        PxVec3 rampVerts[] = {
            PxVec3(-rampScale, 0, -rampScale * 2),
            PxVec3(rampScale, 0, -rampScale * 2),
            PxVec3(rampScale, rampScale * 0.5f, rampScale),
            PxVec3(-rampScale, rampScale * 0.5f, rampScale)
        };
        PxU32 rampIndices[] = {
            0, 1, 2,
            0, 2, 3
        };

        PxTriangleMeshDesc meshDesc;
        meshDesc.points.count = 4;
        meshDesc.points.stride = sizeof(PxVec3);
        meshDesc.points.data = rampVerts;
        meshDesc.triangles.count = 2;
        meshDesc.triangles.stride = 3 * sizeof(PxU32);
        meshDesc.triangles.data = rampIndices;

        PxDefaultMemoryOutputStream buf;
        PxTriangleMeshCookingResult::Enum result;
        if (PxCookTriangleMesh(PxCookingParams(gPhysics->getTolerancesScale()), meshDesc, buf, &result))
        {
            PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
            PxTriangleMesh* triMesh = gPhysics->createTriangleMesh(input);
            if (triMesh)
            {
                PxTriangleMeshGeometry triGeom(triMesh);
                PxShape* rampShape = gPhysics->createShape(triGeom, *gMaterial);
                PxRigidStatic* ramp = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 0, 5)));
                ramp->attachShape(*rampShape);
                gScene->addActor(*ramp);
                rampShape->release();
                std::cout << "  Created triangle mesh ramp" << std::endl;
            }
        }
    }

    // Create falling convex pyramid shapes
    auto createConvexPyramid = [](const PxVec3& position, float scale) -> PxRigidDynamic*
    {
        PxVec3 pyramidVerts[] = {
            PxVec3(0, scale, 0),           // Top
            PxVec3(-scale, 0, -scale),     // Base corners
            PxVec3(scale, 0, -scale),
            PxVec3(scale, 0, scale),
            PxVec3(-scale, 0, scale)
        };

        PxConvexMeshDesc convexDesc;
        convexDesc.points.count = 5;
        convexDesc.points.stride = sizeof(PxVec3);
        convexDesc.points.data = pyramidVerts;
        convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

        PxDefaultMemoryOutputStream buf;
        PxConvexMeshCookingResult::Enum result;
        if (!PxCookConvexMesh(PxCookingParams(gPhysics->getTolerancesScale()), convexDesc, buf, &result))
        {
            std::cerr << "Failed to cook convex mesh" << std::endl;
            return nullptr;
        }

        PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
        PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);
        if (!convexMesh)
        {
            std::cerr << "Failed to create convex mesh" << std::endl;
            return nullptr;
        }

        PxConvexMeshGeometry convexGeom(convexMesh);
        PxRigidDynamic* actor = gPhysics->createRigidDynamic(PxTransform(position));
        PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, convexGeom, *gMaterial);
        if (shape)
        {
            shape->setSimulationFilterData(PxFilterData(1, 1, 0, 0));
        }
        PxRigidBodyExt::updateMassAndInertia(*actor, 1.0f);
        gScene->addActor(*actor);

        return actor;
    };

    // Create multiple falling pyramids
    int pyramidCount = 0;
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            float x = (col - 1.5f) * 2.5f;
            float y = 15.0f + row * 3.0f;
            float z = -3.0f + row * 1.0f;
            if (createConvexPyramid(PxVec3(x, y, z), 0.8f))
            {
                pyramidCount++;
            }
        }
    }
    std::cout << "  Created " << pyramidCount << " convex pyramids" << std::endl;

    // Create a convex tetrahedron
    auto createConvexTetrahedron = [](const PxVec3& position, float scale) -> PxRigidDynamic*
    {
        PxVec3 tetraVerts[] = {
            PxVec3(0, scale * 1.2f, 0),                    // Top
            PxVec3(-scale, -scale * 0.4f, -scale * 0.7f),  // Base corners
            PxVec3(scale, -scale * 0.4f, -scale * 0.7f),
            PxVec3(0, -scale * 0.4f, scale)
        };

        PxConvexMeshDesc convexDesc;
        convexDesc.points.count = 4;
        convexDesc.points.stride = sizeof(PxVec3);
        convexDesc.points.data = tetraVerts;
        convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

        PxDefaultMemoryOutputStream buf;
        PxConvexMeshCookingResult::Enum result;
        if (!PxCookConvexMesh(PxCookingParams(gPhysics->getTolerancesScale()), convexDesc, buf, &result))
        {
            return nullptr;
        }

        PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
        PxConvexMesh* convexMesh = gPhysics->createConvexMesh(input);
        if (!convexMesh) return nullptr;

        PxConvexMeshGeometry convexGeom(convexMesh);
        PxRigidDynamic* actor = gPhysics->createRigidDynamic(PxTransform(position));
        PxRigidActorExt::createExclusiveShape(*actor, convexGeom, *gMaterial);
        PxRigidBodyExt::updateMassAndInertia(*actor, 2.0f);
        gScene->addActor(*actor);

        return actor;
    };

    // Add some tetrahedrons
    int tetraCount = 0;
    for (int i = 0; i < 5; i++)
    {
        if (createConvexTetrahedron(PxVec3(-8.0f + i * 4.0f, 12.0f, -8.0f), 1.0f))
        {
            tetraCount++;
        }
    }
    std::cout << "  Created " << tetraCount << " convex tetrahedrons" << std::endl;

    // Add some standard shapes for reference (these always work)
    std::cout << "  Adding reference boxes and spheres..." << std::endl;
    for (int i = 0; i < 5; i++)
    {
        createBox(PxVec3(-10.0f + i * 5.0f, 8.0f, 8.0f), PxVec3(0.5f, 0.5f, 0.5f), 1.0f);
        createSphere(PxVec3(-10.0f + i * 5.0f, 10.0f, 8.0f), 0.5f, 1.0f);
    }

    std::cout << "Convex Shapes scene created with cooked meshes!" << std::endl;
}

// ========== Test Scene 13: Terrain (Heightfield) ==========
void createSceneTerrain()
{
    std::cout << "Creating Terrain scene (using Heightfield)..." << std::endl;

    // Also create a visible ground box at the base
    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(50.0f, 0.5f, 50.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create heightfield terrain
    const int rows = 32;
    const int cols = 32;
    const float rowScale = 2.0f;
    const float colScale = 2.0f;
    const float heightScale = 0.05f;  // More gentle hills

    // Generate procedural terrain heights (gentler hills)
    std::vector<PxHeightFieldSample> samples(rows * cols);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            // Create gentle hills
            float fx1 = static_cast<float>(c) / cols * 2.0f * 3.14159f;
            float fz1 = static_cast<float>(r) / rows * 2.0f * 3.14159f;

            float h = sinf(fx1) * sinf(fz1) * 40.0f +  // Hills
                      50.0f;                            // Base height (above ground)

            PxHeightFieldSample& sample = samples[r * cols + c];
            sample.height = static_cast<PxI16>(h);  // Direct int16
            sample.materialIndex0 = 0;
            sample.materialIndex1 = 0;
        }
    }

    PxHeightFieldDesc hfDesc;
    hfDesc.nbRows = rows;
    hfDesc.nbColumns = cols;
    hfDesc.samples.data = samples.data();
    hfDesc.samples.stride = sizeof(PxHeightFieldSample);

    PxHeightField* heightField = PxCreateHeightField(hfDesc, gPhysics->getPhysicsInsertionCallback());
    if (heightField)
    {
        PxHeightFieldGeometry hfGeom(heightField, PxMeshGeometryFlags(), heightScale, rowScale, colScale);
        PxShape* hfShape = gPhysics->createShape(hfGeom, *gMaterial);

        // Center the terrain
        PxTransform terrainPose(PxVec3(-rows * rowScale * 0.5f, 0, -cols * colScale * 0.5f));
        PxRigidStatic* terrain = gPhysics->createRigidStatic(terrainPose);
        terrain->attachShape(*hfShape);
        gScene->addActor(*terrain);
        hfShape->release();

        std::cout << "  Created " << rows << "x" << cols << " heightfield terrain" << std::endl;
        std::cout << "  Terrain size: " << (rows * rowScale) << "x" << (cols * colScale) << std::endl;
        std::cout << "  Height range: " << (10 * heightScale) << " to " << (90 * heightScale) << std::endl;
    }
    else
    {
        std::cerr << "  Failed to create heightfield!" << std::endl;
    }

    // Create falling objects on the terrain
    for (int i = 0; i < 20; i++)
    {
        float x = static_cast<float>(rand() % 40 - 20);
        float z = static_cast<float>(rand() % 40 - 20);
        float y = 15.0f + static_cast<float>(rand() % 10);  // Above terrain

        if (rand() % 2 == 0)
        {
            // Create sphere
            createSphere(PxVec3(x, y, z), 0.5f + static_cast<float>(rand() % 100) / 200.0f, 1.0f);
        }
        else
        {
            // Create box
            float size = 0.3f + static_cast<float>(rand() % 100) / 300.0f;
            createBox(PxVec3(x, y, z), PxVec3(size, size, size), 1.0f);
        }
    }

    // Add some capsules too
    for (int i = 0; i < 5; i++)
    {
        float x = static_cast<float>(rand() % 30 - 15);
        float z = static_cast<float>(rand() % 30 - 15);
        float y = 25.0f + static_cast<float>(i * 2);

        PxTransform transform(PxVec3(x, y, z));
        PxRigidDynamic* capsule = gPhysics->createRigidDynamic(transform);
        PxCapsuleGeometry capsuleGeom(0.4f, 0.8f);
        PxShape* shape = PxRigidActorExt::createExclusiveShape(*capsule, capsuleGeom, *gMaterial);
        if (shape)
        {
            shape->setSimulationFilterData(PxFilterData(1, 1, 0, 0));
        }
        PxRigidBodyExt::updateMassAndInertia(*capsule, 1.0f);
        gScene->addActor(*capsule);
    }

    std::cout << "Terrain scene created with rolling objects!" << std::endl;
}

// ========== Test Scene 14: Soft Body (Deformable Volume) ==========
void createSceneSoftBody()
{
    std::cout << "Creating Soft Body scene (using Deformable Volume)..." << std::endl;

    // Check if soft body simulation is available
    if (!gDeformableManager || !gDeformableManager->IsAvailable())
    {
        std::cerr << "ERROR: Soft body simulation requires CUDA/GPU support!" << std::endl;
        std::cerr << "Falling back to regular physics scene..." << std::endl;

        // Create fallback scene with regular objects
        PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
        gScene->addActor(*groundPlane);

        PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(20.0f, 0.5f, 20.0f), *gMaterial);
        PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
        groundBox->attachShape(*groundShape);
        gScene->addActor(*groundBox);
        groundShape->release();

        // Add some regular boxes as placeholder
        for (int i = 0; i < 5; i++)
        {
            createBox(PxVec3(-4.0f + i * 2.0f, 5.0f, 0.0f), PxVec3(0.5f, 0.5f, 0.5f), 1.0f);
        }

        std::cout << "Soft Body scene created (GPU unavailable - showing fallback)" << std::endl;
        return;
    }

    // Create ground
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);

    PxShape* groundShape = gPhysics->createShape(PxBoxGeometry(20.0f, 0.5f, 20.0f), *gMaterial);
    PxRigidStatic* groundBox = gPhysics->createRigidStatic(PxTransform(PxVec3(0, -0.5f, 0)));
    groundBox->attachShape(*groundShape);
    gScene->addActor(*groundBox);
    groundShape->release();

    // Create soft body cubes (jelly cubes)
    DeformableVolumeDef cubeDef;
    cubeDef.shapeType = DeformableVolumeDef::ShapeType::CUBE;
    cubeDef.scale = 1.5f;
    cubeDef.resolution = 8;
    cubeDef.youngsModulus = 5000.0f;  // Relatively soft
    cubeDef.poissonsRatio = 0.45f;
    cubeDef.colorR = 0.2f;
    cubeDef.colorG = 0.8f;
    cubeDef.colorB = 0.4f;

    // Create multiple soft cubes
    for (int i = 0; i < 3; i++)
    {
        cubeDef.posX = -3.0f + i * 3.0f;
        cubeDef.posY = 5.0f + i * 3.0f;
        cubeDef.posZ = 0.0f;

        PxDeformableVolume* volume = gDeformableManager->CreateVolume(cubeDef, gScene);
        if (volume)
        {
            std::cout << "  Created soft cube at (" << cubeDef.posX << ", " << cubeDef.posY << ", " << cubeDef.posZ << ")" << std::endl;
        }
    }

    // Create soft body spheres
    DeformableVolumeDef sphereDef;
    sphereDef.shapeType = DeformableVolumeDef::ShapeType::SPHERE;
    sphereDef.scale = 1.0f;
    sphereDef.resolution = 8;
    sphereDef.youngsModulus = 3000.0f;  // Even softer
    sphereDef.poissonsRatio = 0.48f;
    sphereDef.colorR = 1.0f;
    sphereDef.colorG = 0.4f;
    sphereDef.colorB = 0.4f;

    for (int i = 0; i < 2; i++)
    {
        sphereDef.posX = -1.5f + i * 3.0f;
        sphereDef.posY = 12.0f;
        sphereDef.posZ = 2.0f;

        PxDeformableVolume* volume = gDeformableManager->CreateVolume(sphereDef, gScene);
        if (volume)
        {
            std::cout << "  Created soft sphere at (" << sphereDef.posX << ", " << sphereDef.posY << ", " << sphereDef.posZ << ")" << std::endl;
        }
    }

    // Add some rigid bodies to interact with soft bodies
    for (int i = 0; i < 3; i++)
    {
        createSphere(PxVec3(-2.0f + i * 2.0f, 15.0f, -2.0f), 0.5f, 5.0f);
    }

    std::cout << "Soft Body scene created with deformable volumes!" << std::endl;
}

// ========== Initial scene creation wrapper ==========
void createInitialScene()
{
    // Create the default scene based on gCurrentSceneType
    switch (gCurrentSceneType)
    {
    case SceneType::BASIC_STACK:
        createSceneBasicStack();
        break;
    case SceneType::SHAPES_VARIETY:
        createSceneShapesVariety();
        break;
    case SceneType::FALLING_OBJECTS:
        createSceneFallingObjects();
        break;
    case SceneType::PYRAMID:
        createScenePyramid();
        break;
    case SceneType::STRESS_TEST:
        createSceneStressTest();
        break;
    case SceneType::SNIPPET_HELLO_WORLD:
        createSceneSnippetHelloWorld();
        break;
    case SceneType::PENDULUM:
        createScenePendulum();
        break;
    case SceneType::CHAIN:
        createSceneChain();
        break;
    case SceneType::MECHANISM:
        createSceneMechanism();
        break;
    case SceneType::BREAKABLE_WALL:
        createSceneBreakableWall();
        break;
    case SceneType::ROBOT_ARM:
        createSceneRobotArm();
        break;
    case SceneType::CONVEX_SHAPES:
        createSceneConvexShapes();
        break;
    case SceneType::TERRAIN:
        createSceneTerrain();
        break;
    case SceneType::SOFT_BODY:
        createSceneSoftBody();
        break;
    default:
        createSceneBasicStack();
        break;
    }
}

PxRigidDynamic* createSphere(const PxVec3& position, float radius, float density)
{
    PxTransform transform(position);
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, transform,
        PxSphereGeometry(radius), *gMaterial, density);

    if (!dynamic)
    {
        std::cerr << "Failed to create sphere at (" << position.x << ", "
                  << position.y << ", " << position.z << ")" << std::endl;
        return nullptr;
    }

    // Enable collision notification
    PxShape* shape = nullptr;
    PxU32 shapeCount = dynamic->getShapes(&shape, 1);
    if (shapeCount > 0 && shape)
    {
        shape->setSimulationFilterData(PxFilterData(1, 1, 0, 0));
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
    }

    gScene->addActor(*dynamic);
    return dynamic;
}

PxRigidDynamic* createBox(const PxVec3& position, const PxVec3& halfExtents, float density)
{
    PxTransform transform(position);
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, transform,
        PxBoxGeometry(halfExtents.x, halfExtents.y, halfExtents.z), *gMaterial, density);

    if (!dynamic)
    {
        std::cerr << "Failed to create box at (" << position.x << ", "
                  << position.y << ", " << position.z << ")" << std::endl;
        return nullptr;
    }

    // Enable collision notification
    PxShape* shape = nullptr;
    PxU32 shapeCount = dynamic->getShapes(&shape, 1);
    if (shapeCount > 0 && shape)
    {
        shape->setSimulationFilterData(PxFilterData(1, 1, 0, 0));
        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
    }

    gScene->addActor(*dynamic);
    return dynamic;
}

void switchScene(SceneType newType)
{
    const char* sceneNames[] = {
        "Basic Stack", "Shapes Variety", "Falling Objects",
        "Pyramid", "Stress Test", "Snippet HelloWorld",
        "Pendulum", "Chain", "Mechanism", "Breakable Wall", "Robot Arm", "Convex Shapes", "Terrain", "Soft Body"
    };

    std::cout << "\n========================================" << std::endl;
    std::cout << "SWITCHING TO SCENE: " << sceneNames[static_cast<int>(newType)] << std::endl;
    std::cout << "========================================" << std::endl;

    if (newType != gCurrentSceneType)
    {
        gCurrentSceneType = newType;
    }

    if (!gScene) return;

    std::cout << "Clearing existing scene..." << std::endl;

    // Release all joints first (before actors)
    for (PxJoint* joint : gJoints)
    {
        if (joint)
        {
            joint->release();
        }
    }
    gJoints.clear();

    // Release all articulations (before actors, as links are part of articulations)
    for (PxArticulationReducedCoordinate* articulation : gArticulations)
    {
        if (articulation)
        {
            gScene->removeArticulation(*articulation);
            articulation->release();
        }
    }
    gArticulations.clear();

    // Release all deformable volumes (soft bodies)
    if (gDeformableManager)
    {
        gDeformableManager->ReleaseAll(gScene);
    }

    // Get all actors in the scene
    PxU32 nbActors = gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if (nbActors > 0)
    {
        std::vector<PxRigidActor*> actors(nbActors);
        gScene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC,
                          reinterpret_cast<PxActor**>(&actors[0]), nbActors);

        // Remove all actors from the scene
        for (PxU32 i = 0; i < nbActors; i++)
        {
            gScene->removeActor(*actors[i]);
            actors[i]->release();
        }
    }

    // Clear renderer visualization data
    if (gRenderer)
    {
        gRenderer->ClearCollisionPoints();
        gRenderer->ClearTrajectories();
    }

    // Create new scene
    createInitialScene();

    // Pause the simulation so user can see the initial setup
    if (gController && !gController->IsPaused())
    {
        gController->TogglePause();
        std::cout << "Simulation paused to show initial scene setup" << std::endl;
    }

    // Set camera position appropriate for the scene
    if (gRenderer)
    {
        std::cout << "Setting camera position for scene..." << std::endl;
        switch (newType)
        {
        case SceneType::BASIC_STACK:
            gRenderer->GetCamera().SetPosition(0.0f, 15.0f, 30.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 10.0f, 0.0f);
            std::cout << "  Camera: Pos(0, 15, 30) -> LookAt(0, 10, 0)" << std::endl;
            break;
        case SceneType::SHAPES_VARIETY:
            gRenderer->GetCamera().SetPosition(0.0f, 20.0f, 40.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 8.0f, 0.0f);
            std::cout << "  Camera: Pos(0, 20, 40) -> LookAt(0, 8, 0)" << std::endl;
            break;
        case SceneType::FALLING_OBJECTS:
            gRenderer->GetCamera().SetPosition(0.0f, 35.0f, 50.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 25.0f, 0.0f);
            std::cout << "  Camera: Pos(0, 35, 50) -> LookAt(0, 25, 0)" << std::endl;
            break;
        case SceneType::PYRAMID:
            gRenderer->GetCamera().SetPosition(30.0f, 25.0f, 30.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 10.0f, 0.0f);
            std::cout << "  Camera: Pos(30, 25, 30) -> LookAt(0, 10, 0)" << std::endl;
            break;
        case SceneType::STRESS_TEST:
            gRenderer->GetCamera().SetPosition(0.0f, 80.0f, 80.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 0.0f, 0.0f);
            std::cout << "  Camera: Pos(0, 80, 80) -> LookAt(0, 0, 0)" << std::endl;
            break;
        case SceneType::SNIPPET_HELLO_WORLD:
            gRenderer->GetCamera().SetPosition(0.0f, 30.0f, 60.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 10.0f, -20.0f);
            std::cout << "  Camera: Pos(0, 30, 60) -> LookAt(0, 10, -20)" << std::endl;
            break;
        case SceneType::PENDULUM:
            gRenderer->GetCamera().SetPosition(20.0f, 15.0f, 30.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 10.0f, 0.0f);
            std::cout << "  Camera: Pos(20, 15, 30) -> LookAt(0, 10, 0)" << std::endl;
            break;
        case SceneType::CHAIN:
            gRenderer->GetCamera().SetPosition(15.0f, 10.0f, 30.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 10.0f, 0.0f);
            std::cout << "  Camera: Pos(15, 10, 30) -> LookAt(0, 10, 0)" << std::endl;
            break;
        case SceneType::MECHANISM:
            gRenderer->GetCamera().SetPosition(10.0f, 10.0f, 25.0f);
            gRenderer->GetCamera().SetLookAt(5.0f, 5.0f, 0.0f);
            std::cout << "  Camera: Pos(10, 10, 25) -> LookAt(5, 5, 0)" << std::endl;
            break;
        case SceneType::BREAKABLE_WALL:
            gRenderer->GetCamera().SetPosition(-15.0f, 5.0f, 20.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 2.0f, 0.0f);
            std::cout << "  Camera: Pos(-15, 5, 20) -> LookAt(0, 2, 0)" << std::endl;
            break;
        case SceneType::ROBOT_ARM:
            gRenderer->GetCamera().SetPosition(5.0f, 8.0f, 15.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 3.0f, 0.0f);
            std::cout << "  Camera: Pos(5, 8, 15) -> LookAt(0, 3, 0)" << std::endl;
            break;
        case SceneType::CONVEX_SHAPES:
            gRenderer->GetCamera().SetPosition(15.0f, 15.0f, 30.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 5.0f, 0.0f);
            std::cout << "  Camera: Pos(15, 15, 30) -> LookAt(0, 5, 0)" << std::endl;
            break;
        case SceneType::TERRAIN:
            gRenderer->GetCamera().SetPosition(40.0f, 30.0f, 40.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 5.0f, 0.0f);
            std::cout << "  Camera: Pos(40, 30, 40) -> LookAt(0, 5, 0)" << std::endl;
            break;
        case SceneType::SOFT_BODY:
            gRenderer->GetCamera().SetPosition(10.0f, 10.0f, 20.0f);
            gRenderer->GetCamera().SetLookAt(0.0f, 5.0f, 0.0f);
            std::cout << "  Camera: Pos(10, 10, 20) -> LookAt(0, 5, 0)" << std::endl;
            break;
        }

        // Verify camera position was set
        XMFLOAT3 camPos = gRenderer->GetCamera().GetPosition();
        XMFLOAT3 camTarget = gRenderer->GetCamera().GetTarget();
        std::cout << "  Camera VERIFIED: Pos(" << camPos.x << ", " << camPos.y << ", " << camPos.z
                  << ") Target(" << camTarget.x << ", " << camTarget.y << ", " << camTarget.z << ")" << std::endl;
    }

    // Reapply simulation parameters
    if (gController)
    {
        gController->ApplyParametersToScene(gScene, gMaterial);
    }

    std::cout << "Scene switch complete!" << std::endl;
}

void resetScene()
{
    // Reset current scene
    switchScene(gCurrentSceneType);
}

void stepPhysics(float deltaTime)
{
    gScene->simulate(deltaTime);
    gScene->fetchResults(true);

    // Update soft body render meshes after simulation
    if (gDeformableManager && gDeformableManager->IsAvailable())
    {
        gDeformableManager->UpdateRenderMeshes();
    }
}

void cleanupPhysics()
{
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PxCloseExtensions();  // Close extensions
    PX_RELEASE(gPhysics);
    if (gCudaContextManager)
    {
        gCudaContextManager->release();
        gCudaContextManager = nullptr;
    }
    PX_RELEASE(gFoundation);
    std::cout << "PhysX cleaned up." << std::endl;
}

// Main entry point
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    // Allocate a console for debugging output
    AllocConsole();
    FILE* fpStdout = nullptr;
    FILE* fpStderr = nullptr;
    freopen_s(&fpStdout, "CONOUT$", "w", stdout);
    freopen_s(&fpStderr, "CONOUT$", "w", stderr);
    SetConsoleTitleW(L"PhysX Research Tool Console");

    // Parse command line arguments
    int argc = __argc;
    char** argv = __argv;

    if (!gArgs.Parse(argc, argv))
    {
        std::cerr << "Failed to parse command line arguments" << std::endl;
        return -1;
    }

    // Show help if requested
    if (gArgs.help)
    {
        gArgs.PrintHelp();
        return 0;
    }

    // Validate arguments
    std::string validationError;
    if (!gArgs.Validate(validationError))
    {
        std::cerr << "Invalid arguments: " << validationError << std::endl;
        gArgs.PrintHelp();
        return -1;
    }

    // Print mode
    if (gArgs.headless)
    {
        std::cout << "=== PhysX Research Tool - HEADLESS MODE ===" << std::endl;
        std::cout << "Running without rendering for maximum performance" << std::endl;
    }
    else
    {
        std::cout << "=== PhysX with DirectX 12 Visualization ===" << std::endl;
    }
    std::cout << "Starting application..." << std::endl;

    // Create output directory if it doesn't exist
    if (gArgs.headless)
    {
        std::filesystem::create_directories(gArgs.outputDir);
        std::cout << "Output directory: " << gArgs.outputDir << std::endl;
    }

    try
    {
    // Create window (only in GUI mode)
    if (!gArgs.headless)
    {
        std::cout << "Creating window..." << std::endl;
        if (!CreateAppWindow(hInstance))
        {
            std::cerr << "Failed to create window!" << std::endl;
            std::cerr << "Error code: " << GetLastError() << std::endl;
            return -1;
        }
        std::cout << "Window created successfully!" << std::endl;
    }

    // Initialize PhysX
    std::cout << "Initializing PhysX..." << std::endl;
    if (!initPhysics())
    {
        std::cerr << "PhysX initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "PhysX initialization complete!" << std::endl;

    // Create renderer (only in GUI mode)
    if (!gArgs.headless)
    {
        std::cout << "Creating DirectX 12 renderer..." << std::endl;
        gRenderer = std::make_unique<DX12Renderer>();
        std::cout << "Initializing DirectX 12 renderer..." << std::endl;
        if (!gRenderer->Initialize(gHwnd, WINDOW_WIDTH, WINDOW_HEIGHT))
        {
            std::cerr << "Failed to initialize DirectX 12 renderer!" << std::endl;
            cleanupPhysics();
            return -1;
        }
        std::cout << "DirectX 12 renderer initialized successfully!" << std::endl;

        // Initialize ImGui
        gRenderer->InitializeImGui(gHwnd);
    }

    // Initialize simulation research systems
    std::cout << "Initializing simulation research systems..." << std::endl;
    gRecorder = std::make_unique<SimulationRecorder>();
    gProfiler = std::make_unique<PerformanceProfiler>();
    gController = std::make_unique<ExperimentController>();
    std::cout << "Simulation systems initialized!" << std::endl;

    // Set random seed for deterministic simulation
    if (gArgs.randomSeed > 0)
    {
        srand(gArgs.randomSeed);
        std::cout << "Random seed set to: " << gArgs.randomSeed << " (deterministic mode)" << std::endl;
    }
    else
    {
        srand(static_cast<unsigned int>(time(nullptr)));
        std::cout << "Random seed: using current time (non-deterministic)" << std::endl;
    }

    // Set scene type from command line
    if (gArgs.sceneType == "basic_stack")
        gCurrentSceneType = SceneType::BASIC_STACK;
    else if (gArgs.sceneType == "shapes_variety")
        gCurrentSceneType = SceneType::SHAPES_VARIETY;
    else if (gArgs.sceneType == "falling_objects")
        gCurrentSceneType = SceneType::FALLING_OBJECTS;
    else if (gArgs.sceneType == "pyramid")
        gCurrentSceneType = SceneType::PYRAMID;
    else if (gArgs.sceneType == "stress_test")
        gCurrentSceneType = SceneType::STRESS_TEST;
    else if (gArgs.sceneType == "snippet_hello_world")
        gCurrentSceneType = SceneType::SNIPPET_HELLO_WORLD;
    else if (!gArgs.sceneType.empty())
    {
        std::cerr << "Warning: Unknown scene type '" << gArgs.sceneType << "', using basic_stack" << std::endl;
        gCurrentSceneType = SceneType::BASIC_STACK;
    }

    // Create scene objects
    std::cout << "Creating scene: " << gArgs.sceneType << std::endl;
    createInitialScene();
    std::cout << "Initial scene created." << std::endl;

    // Apply simulation parameters (gravity, friction, etc.) to the scene
    gController->ApplyParametersToScene(gScene, gMaterial);
    std::cout << "Simulation parameters applied to scene." << std::endl;

    // Headless mode: Start immediately and enable recording
    if (gArgs.headless)
    {
        // Start simulation immediately in headless mode
        std::cout << "Headless mode: Simulation starting immediately" << std::endl;

        // Start recording
        gRecorder->StartRecording();
        std::cout << "Recording started" << std::endl;

        // Print simulation parameters
        std::cout << "\nSimulation Configuration:" << std::endl;
        if (gArgs.maxFrames > 0)
            std::cout << "  Max frames: " << gArgs.maxFrames << std::endl;
        if (gArgs.maxTime > 0.0f)
            std::cout << "  Max time: " << gArgs.maxTime << " seconds" << std::endl;
        std::cout << "  Record interval: every " << gArgs.recordInterval << " frame(s)" << std::endl;
        if (gArgs.randomSeed > 0)
            std::cout << "  Random seed: " << gArgs.randomSeed << " (deterministic)" << std::endl;
    }
    else
    {
        // GUI mode: Start paused so user can see initial setup
        gController->TogglePause();
        std::cout << "Simulation PAUSED - Press 'Paused' checkbox in UI to start" << std::endl;

        // Enable some default visualizations (reduced for better performance)
        gRenderer->SetInstancingEnabled(true);  // Enable instanced rendering for performance
        gRenderer->SetVelocityVectorVisualization(false);  // Disable by default (can be enabled in UI)
        gRenderer->SetCollisionVisualization(true);
        gRenderer->SetTrajectoryVisualization(false);  // Disable by default (heavy)
        gRenderer->SetForceVisualization(false);  // Disable by default (heavy)
        std::cout << "Default visualizations configured (instancing enabled)." << std::endl;

        std::cout << "\nControls:" << std::endl;
        std::cout << "  WASD - Move camera target" << std::endl;
        std::cout << "  Q/E - Move camera target up/down" << std::endl;
        std::cout << "  Right Mouse Button - Orbit camera around target" << std::endl;
        std::cout << "  Middle Mouse Button - Pan camera" << std::endl;
        std::cout << "  Mouse Wheel - Zoom in/out" << std::endl;
        std::cout << "  Shift + Mouse - Camera control over UI windows" << std::endl;
        std::cout << "  ESC - Exit" << std::endl;
        std::cout << "\nNote: Camera controls work even when simulation is paused." << std::endl;
    }

    std::cout << "\n=== Entering main loop ===" << std::endl;
    std::cout.flush();

    // Main loop
    MSG msg = {};
    const float timeStep = 1.0f / 60.0f;
    LARGE_INTEGER frequency, lastTime, currentTime;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&lastTime);

    int frameCount = 0;
    int recordedFrameCount = 0;

    float accumulatedTime = 0.0f;

    while (gRunning)
    {
        // Begin frame profiling
        if (gArgs.enableProfiling)
        {
            gProfiler->BeginFrame();
        }

        // Process messages (only in GUI mode)
        if (!gArgs.headless)
        {
            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        }

        // Calculate delta time
        QueryPerformanceCounter(&currentTime);
        float deltaTime = static_cast<float>(currentTime.QuadPart - lastTime.QuadPart) / frequency.QuadPart;
        lastTime = currentTime;

        // Check termination conditions for headless mode
        if (gArgs.headless)
        {
            if (gArgs.maxFrames > 0 && frameCount >= gArgs.maxFrames)
            {
                std::cout << "Reached maximum frames (" << gArgs.maxFrames << "), stopping simulation" << std::endl;
                break;
            }
            if (gArgs.maxTime > 0.0f && accumulatedTime >= gArgs.maxTime)
            {
                std::cout << "Reached maximum time (" << gArgs.maxTime << "s), stopping simulation" << std::endl;
                break;
            }
        }

        // Get adjusted timestep from controller (handles pause, time scale, step mode)
        float adjustedTimeStep = gController->GetAdjustedTimestep(timeStep);

        // Physics simulation
        if (adjustedTimeStep > 0.0f || gController->CanStep())
        {
            if (gArgs.enableProfiling)
            {
                gProfiler->BeginPhysics();
            }

            stepPhysics(adjustedTimeStep);

            if (gArgs.enableProfiling)
            {
                gProfiler->EndPhysics();
            }

            // Record frame data if recording is active (respecting record interval)
            accumulatedTime += adjustedTimeStep;
            if (gRecorder->IsRecording() && (frameCount % gArgs.recordInterval == 0))
            {
                gRecorder->RecordFrame(gScene, accumulatedTime);
                recordedFrameCount++;
            }

            // Clear step request after stepping
            if (gController->CanStep())
            {
                gController->ClearStepRequest();
            }
        }

        // Handle scene reset request
        if (gController->IsResetRequested())
        {
            resetScene();
            gController->ClearResetRequest();
            accumulatedTime = 0.0f;  // Reset simulation time
            gLastSpawnTime = 0.0f;  // Reset spawn timer
        }

        // Continuous object spawning
        if (gEnableContinuousSpawn && (accumulatedTime - gLastSpawnTime) >= gSpawnInterval)
        {
            // Spawn a new sphere at random position
            float x = ((rand() % 200) - 100) / 20.0f;  // -5 to 5
            float z = ((rand() % 200) - 100) / 20.0f;  // -5 to 5
            createSphere(PxVec3(x, 20.0f, z), 0.5f);
            gLastSpawnTime = accumulatedTime;
        }

        // Update camera (only in GUI mode)
        if (!gArgs.headless && gRenderer)
        {
            gRenderer->GetCamera().Update(deltaTime);
        }

        // Update trajectories (only in GUI mode)
        if (!gArgs.headless && gRenderer)
        {
            gRenderer->UpdateTrajectories(gScene, deltaTime);
        }

        // Rendering (only in GUI mode)
        if (!gArgs.headless && gRenderer)
        {
            if (gArgs.enableProfiling)
            {
                gProfiler->BeginRendering();
            }

            gRenderer->BeginFrame();

            // Use instanced rendering for better performance with large object counts
            if (gRenderer->GetInstancingEnabled())
            {
                gRenderer->RenderPhysXSceneInstanced(gScene);
            }
            else
            {
                gRenderer->RenderPhysXScene(gScene);
            }

            gRenderer->RenderImGui(gScene, deltaTime, gCudaContextManager,
                                  gRecorder.get(), gProfiler.get(), gController.get(), gMaterial);
            gRenderer->EndFrame();

            if (gArgs.enableProfiling)
            {
                gProfiler->EndRendering();
            }
        }

        // End frame profiling
        if (gArgs.enableProfiling)
        {
            gProfiler->EndFrame();
        }

        // Log progress
        frameCount++;

        // Headless mode: log every 600 frames (10 seconds), show progress
        if (gArgs.headless && frameCount % 600 == 0)
        {
            std::cout << "Frame " << frameCount << " / ";
            if (gArgs.maxFrames > 0)
                std::cout << gArgs.maxFrames;
            else
                std::cout << "unlimited";
            std::cout << " - Time: " << accumulatedTime << "s";
            if (gArgs.maxTime > 0.0f)
                std::cout << " / " << gArgs.maxTime << "s";
            if (gArgs.enableProfiling)
                std::cout << " - FPS: " << gProfiler->GetCurrentFPS();
            std::cout << std::endl;
        }
        // GUI mode: log every 300 frames (5 seconds)
        else if (!gArgs.headless && frameCount % 300 == 0 && gArgs.enableProfiling)
        {
            std::cout << "Frame " << frameCount << " - FPS: " << gProfiler->GetCurrentFPS() << std::endl;
        }
    }

    // Cleanup
    std::cout << "\n=== Starting cleanup ===" << std::endl;

    // Export data for headless mode
    if (gArgs.headless && gRecorder)
    {
        std::cout << "\nExporting simulation data..." << std::endl;
        std::cout << "  Total frames simulated: " << frameCount << std::endl;
        std::cout << "  Total frames recorded: " << recordedFrameCount << std::endl;
        std::cout << "  Total simulation time: " << accumulatedTime << "s" << std::endl;

        // Stop recording if still active
        if (gRecorder->IsRecording())
        {
            gRecorder->StopRecording();
        }

        // Generate filename
        std::string baseFilename = gArgs.experimentName.empty()
            ? "experiment"
            : gArgs.experimentName;

        // Add timestamp if no custom name
        if (gArgs.experimentName.empty())
        {
            time_t now = time(nullptr);
            char timestamp[32];
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", localtime(&now));
            baseFilename += "_" + std::string(timestamp);
        }

        // Export to CSV
        std::string csvPath = gArgs.outputDir + baseFilename + ".csv";
        if (gRecorder->ExportToCSV(csvPath))
        {
            std::cout << "  [OK] CSV exported: " << csvPath << std::endl;
        }
        else
        {
            std::cerr << "  [FAIL] Failed to export CSV" << std::endl;
        }

        // Export to JSON
        std::string jsonPath = gArgs.outputDir + baseFilename + ".json";
        if (gRecorder->ExportToJSON(jsonPath))
        {
            std::cout << "  [OK] JSON exported: " << jsonPath << std::endl;
        }
        else
        {
            std::cerr << "  [FAIL] Failed to export JSON" << std::endl;
        }

        std::cout << "Data export complete!" << std::endl;
    }

    // Cleanup renderer (only if in GUI mode)
    if (gRenderer)
    {
        std::cout << "Shutting down ImGui..." << std::endl;
        gRenderer->ShutdownImGui();
        std::cout << "Shutting down DirectX 12 renderer..." << std::endl;
        gRenderer->Shutdown();
        gRenderer.reset();
        std::cout << "Renderer shutdown complete." << std::endl;
    }

    std::cout << "Cleaning up PhysX..." << std::endl;
    cleanupPhysics();

    std::cout << "Application terminated successfully." << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n!!! EXCEPTION CAUGHT !!!" << std::endl;
        std::cerr << "Exception: " << e.what() << std::endl;
        std::cerr << "\nPress Enter to close..." << std::endl;
        std::cin.get();
        FreeConsole();
        return -1;
    }
    catch (...)
    {
        std::cerr << "\n!!! UNKNOWN EXCEPTION CAUGHT !!!" << std::endl;
        std::cerr << "\nPress Enter to close..." << std::endl;
        std::cin.get();
        FreeConsole();
        return -1;
    }

    std::cout << "\nPress Enter to close..." << std::endl;
    std::cin.get();

    FreeConsole();
    return 0;
}
