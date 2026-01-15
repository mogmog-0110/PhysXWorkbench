#include <PxPhysicsAPI.h>
#include <iostream>
#include <vector>
#include <windows.h>
#include "dx12/DX12Renderer.h"
#include "imgui.h"
#include "imgui_impl_win32.h"

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
const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;
bool gRunning = true;

// Mouse tracking
bool gMouseCaptured = false;
POINT gLastMousePos;

// Forward declarations
bool initPhysics();
void stepPhysics(float deltaTime);
void cleanupPhysics();
PxRigidDynamic* createBox(const PxVec3& position, const PxVec3& halfExtents, float density = 1.0f);
PxRigidDynamic* createSphere(const PxVec3& position, float radius, float density = 1.0f);
PxRigidStatic* createGround();

// Window procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    // Let ImGui handle the message first
    if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam))
        return true;

    switch (uMsg)
    {
    case WM_DESTROY:
        gRunning = false;
        PostQuitMessage(0);
        return 0;

    case WM_KEYDOWN:
        // Only handle camera input if ImGui doesn't want keyboard input
        if (gRenderer && !ImGui::GetIO().WantCaptureKeyboard)
        {
            gRenderer->GetCamera().HandleInput(wParam);
        }

        if (wParam == VK_ESCAPE)
        {
            gRunning = false;
            PostQuitMessage(0);
        }
        return 0;

    case WM_RBUTTONDOWN:
        // Only capture mouse if ImGui doesn't want mouse input
        if (!ImGui::GetIO().WantCaptureMouse)
        {
            gMouseCaptured = true;
            GetCursorPos(&gLastMousePos);
            ScreenToClient(hwnd, &gLastMousePos);
            SetCapture(hwnd);
            ShowCursor(FALSE);
        }
        return 0;

    case WM_RBUTTONUP:
        if (gMouseCaptured)
        {
            gMouseCaptured = false;
            ReleaseCapture();
            ShowCursor(TRUE);
        }
        return 0;

    case WM_MOUSEMOVE:
        if (gMouseCaptured && gRenderer && !ImGui::GetIO().WantCaptureMouse)
        {
            POINT currentPos;
            GetCursorPos(&currentPos);
            ScreenToClient(hwnd, &currentPos);

            int dx = currentPos.x - gLastMousePos.x;
            int dy = currentPos.y - gLastMousePos.y;

            gRenderer->GetCamera().HandleMouse(dx, dy);

            gLastMousePos = currentPos;
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

    gScene = gPhysics->createScene(sceneDesc);

    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    std::cout << "PhysX initialized successfully!" << std::endl;
    return true;
}

PxRigidStatic* createGround()
{
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
    gScene->addActor(*groundPlane);
    return groundPlane;
}

PxRigidDynamic* createBox(const PxVec3& position, const PxVec3& halfExtents, float density)
{
    PxTransform transform(position);
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, transform,
        PxBoxGeometry(halfExtents), *gMaterial, density);
    gScene->addActor(*dynamic);
    return dynamic;
}

PxRigidDynamic* createSphere(const PxVec3& position, float radius, float density)
{
    PxTransform transform(position);
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, transform,
        PxSphereGeometry(radius), *gMaterial, density);
    gScene->addActor(*dynamic);
    return dynamic;
}

void stepPhysics(float deltaTime)
{
    gScene->simulate(deltaTime);
    gScene->fetchResults(true);
}

void cleanupPhysics()
{
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
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
    SetConsoleTitleW(L"PhysX DirectX 12 Debug Console");

    std::cout << "=== PhysX with DirectX 12 Visualization ===" << std::endl;
    std::cout << "Starting application..." << std::endl;

    try
    {
    // Create window
    std::cout << "Creating window..." << std::endl;
    if (!CreateAppWindow(hInstance))
    {
        std::cerr << "Failed to create window!" << std::endl;
        std::cerr << "Error code: " << GetLastError() << std::endl;
        return -1;
    }
    std::cout << "Window created successfully!" << std::endl;

    // Initialize PhysX
    std::cout << "Initializing PhysX..." << std::endl;
    if (!initPhysics())
    {
        std::cerr << "PhysX initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "PhysX initialization complete!" << std::endl;

    // Create renderer
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

    // Create scene objects
    createGround();
    std::cout << "Ground plane created." << std::endl;

    std::vector<PxRigidDynamic*> objects;

    // Create boxes
    objects.push_back(createBox(PxVec3(0, 10, 0), PxVec3(0.5f, 0.5f, 0.5f)));
    objects.push_back(createBox(PxVec3(2, 15, 0), PxVec3(0.5f, 0.5f, 0.5f)));
    objects.push_back(createBox(PxVec3(-2, 20, 0), PxVec3(0.5f, 0.5f, 0.5f)));

    // Create spheres
    objects.push_back(createSphere(PxVec3(1, 25, 1), 0.5f));
    objects.push_back(createSphere(PxVec3(-1, 30, -1), 0.5f));

    // Create large box
    objects.push_back(createBox(PxVec3(0, 35, 0), PxVec3(1.0f, 1.0f, 1.0f)));

    std::cout << "Created " << objects.size() << " dynamic objects." << std::endl;
    std::cout << "\nControls:" << std::endl;
    std::cout << "  WASD - Move camera" << std::endl;
    std::cout << "  Q/E - Move camera up/down" << std::endl;
    std::cout << "  Right Mouse Button - Look around" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;
    std::cout << "\n=== Entering main loop ===" << std::endl;
    std::cout.flush();

    // Main loop
    MSG msg = {};
    const float timeStep = 1.0f / 60.0f;
    LARGE_INTEGER frequency, lastTime, currentTime;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&lastTime);

    int frameCount = 0;

    while (gRunning)
    {
        // Process messages
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        // Calculate delta time
        QueryPerformanceCounter(&currentTime);
        float deltaTime = static_cast<float>(currentTime.QuadPart - lastTime.QuadPart) / frequency.QuadPart;
        lastTime = currentTime;

        // Physics simulation
        stepPhysics(timeStep);

        // Rendering
        gRenderer->BeginFrame();
        gRenderer->RenderPhysXScene(gScene);
        gRenderer->RenderImGui(gScene, deltaTime, gCudaContextManager);
        gRenderer->EndFrame();

        // Log every 60 frames (once per second at 60 FPS)
        frameCount++;
        if (frameCount % 60 == 0)
        {
            std::cout << "Frame " << frameCount << " rendered successfully." << std::endl;
        }
    }

    // Cleanup
    std::cout << "\n=== Starting cleanup ===" << std::endl;
    std::cout << "Shutting down ImGui..." << std::endl;
    gRenderer->ShutdownImGui();
    std::cout << "Shutting down DirectX 12 renderer..." << std::endl;
    gRenderer->Shutdown();
    gRenderer.reset();
    std::cout << "Renderer shutdown complete." << std::endl;

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
