#include "DX12Renderer.h"
#include "d3dx12.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <cfloat>
#include <algorithm>
#include "imgui.h"
#include "imgui_internal.h"  // For DockBuilder functions
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "../simulation/ExperimentController.h"
#include "../simulation/SimulationRecorder.h"
#include "../simulation/PerformanceProfiler.h"
#include "../simulation/DeformableVolumeManager.h"

// External variables from main
extern bool gEnableContinuousSpawn;
extern float gSpawnInterval;
extern std::unique_ptr<DeformableVolumeManager> gDeformableManager;

// Scene selection (from main_dx12.cpp)
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
extern SceneType gCurrentSceneType;
void switchScene(SceneType newType);

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")

using namespace physx;

// Helper function to throw on HRESULT failure
inline void ThrowIfFailed(HRESULT hr)
{
    if (FAILED(hr))
    {
        throw std::runtime_error("DirectX 12 operation failed");
    }
}

//=============================================================================
// Camera Implementation
//=============================================================================

Camera::Camera()
    : position(0.0f, 30.0f, 60.0f)
    , target(0.0f, 5.0f, 0.0f)       // Look at scene center
    , up(0.0f, 1.0f, 0.0f)
    , distance(70.0f)                 // Distance from target
    , yaw(0.0f)
    , pitch(0.4f)                     // Look down at scene
    , moveSpeed(30.0f)
    , rotateSpeed(0.003f)             // Reduced for smoother rotation
    , panSpeed(0.1f)                  // Reduced for finer control
    , zoomSpeed(5.0f)                 // Increased for responsive zoom
    , fov(XM_PIDIV4)
    , aspectRatio(16.0f / 9.0f)
    , nearPlane(0.1f)
    , farPlane(1000.0f)
    , keyW(false), keyS(false), keyA(false), keyD(false), keyQ(false), keyE(false)
{
    // Initialize position from orbit parameters
    UpdatePositionFromOrbit();
}

void Camera::SetPosition(float x, float y, float z)
{
    position = XMFLOAT3(x, y, z);
}

void Camera::SetLookAt(float x, float y, float z)
{
    target = XMFLOAT3(x, y, z);

    // Calculate distance, yaw and pitch from position and target
    XMVECTOR posVec = XMLoadFloat3(&position);
    XMVECTOR targetVec = XMLoadFloat3(&target);
    XMVECTOR toCamera = XMVectorSubtract(posVec, targetVec);  // From target to camera

    // Calculate distance from target to camera
    XMVECTOR distanceVec = XMVector3Length(toCamera);
    XMStoreFloat(&distance, distanceVec);

    // Normalize direction (from target to camera)
    XMVECTOR direction = XMVector3Normalize(toCamera);

    XMFLOAT3 dir;
    XMStoreFloat3(&dir, direction);

    // Calculate yaw (rotation around Y axis) - angle from +Z axis
    yaw = atan2f(dir.x, dir.z);

    // Calculate pitch (elevation angle)
    float horizontalDist = sqrtf(dir.x * dir.x + dir.z * dir.z);
    pitch = atan2f(dir.y, horizontalDist);
}

void Camera::Update(float deltaTime)
{
    // Unity-style orbit camera: WASD/QE moves the target, camera follows
    // Calculate camera's right and forward vectors (horizontal plane only for intuitive movement)
    XMFLOAT3 forward;
    forward.x = sinf(yaw);
    forward.y = 0.0f;
    forward.z = cosf(yaw);

    XMVECTOR forwardVec = XMLoadFloat3(&forward);
    forwardVec = XMVector3Normalize(forwardVec);

    XMVECTOR rightVec = XMVector3Cross(forwardVec, XMLoadFloat3(&up));
    rightVec = XMVector3Normalize(rightVec);

    XMVECTOR upVec = XMLoadFloat3(&up);

    // Move target with WASDQE (camera orbit center moves)
    XMVECTOR targetVec = XMLoadFloat3(&target);
    float speed = moveSpeed * deltaTime;

    // W/S: Move target forward/backward (relative to camera view)
    if (keyW) { targetVec = XMVectorAdd(targetVec, XMVectorScale(forwardVec, speed)); }
    if (keyS) { targetVec = XMVectorSubtract(targetVec, XMVectorScale(forwardVec, speed)); }
    // A/D: Move target left/right
    if (keyD) { targetVec = XMVectorAdd(targetVec, XMVectorScale(rightVec, speed)); }
    if (keyA) { targetVec = XMVectorSubtract(targetVec, XMVectorScale(rightVec, speed)); }
    // Q/E: Move target up/down
    if (keyE) { targetVec = XMVectorAdd(targetVec, XMVectorScale(upVec, speed)); }
    if (keyQ) { targetVec = XMVectorSubtract(targetVec, XMVectorScale(upVec, speed)); }

    XMStoreFloat3(&target, targetVec);

    // Update camera position based on orbit around target
    UpdatePositionFromOrbit();
}

void Camera::HandleInput(WPARAM wParam)
{
    switch (wParam)
    {
    case 'W': keyW = true; break;
    case 'S': keyS = true; break;
    case 'A': keyA = true; break;
    case 'D': keyD = true; break;
    case 'Q': keyQ = true; break;
    case 'E': keyE = true; break;
    }
}

void Camera::HandleKeyUp(WPARAM wParam)
{
    switch (wParam)
    {
    case 'W': keyW = false; break;
    case 'S': keyS = false; break;
    case 'A': keyA = false; break;
    case 'D': keyD = false; break;
    case 'Q': keyQ = false; break;
    case 'E': keyE = false; break;
    }
}

void Camera::HandleMouseOrbit(int dx, int dy)
{
    // Unity-style: drag to orbit around target
    yaw += dx * rotateSpeed;  // Horizontal mouse = horizontal orbit
    pitch += dy * rotateSpeed; // Vertical mouse = vertical orbit

    // Clamp pitch to avoid flipping over
    const float maxPitch = XM_PI / 2.0f - 0.05f;
    const float minPitch = -XM_PI / 2.0f + 0.05f;
    if (pitch > maxPitch) pitch = maxPitch;
    if (pitch < minPitch) pitch = minPitch;

    // Immediately update camera position
    UpdatePositionFromOrbit();
}

void Camera::HandleMousePan(int dx, int dy)
{
    // Unity-style: pan moves both camera and target together
    // Calculate right and up vectors relative to camera view
    float cosPitch = cosf(pitch);
    XMFLOAT3 forward;
    forward.x = -sinf(yaw) * cosPitch;
    forward.y = -sinf(pitch);
    forward.z = -cosf(yaw) * cosPitch;

    XMVECTOR forwardVec = XMLoadFloat3(&forward);
    forwardVec = XMVector3Normalize(forwardVec);
    XMVECTOR rightVec = XMVector3Cross(XMLoadFloat3(&up), forwardVec);
    rightVec = XMVector3Normalize(rightVec);

    // Calculate camera-relative up (perpendicular to view direction)
    XMVECTOR camUpVec = XMVector3Cross(forwardVec, rightVec);
    camUpVec = XMVector3Normalize(camUpVec);

    // Scale pan amount by distance for consistent feel at all zoom levels
    float scaledPan = panSpeed * distance * 0.01f;

    // Move target (camera follows via UpdatePositionFromOrbit)
    XMVECTOR targetVec = XMLoadFloat3(&target);
    targetVec = XMVectorAdd(targetVec, XMVectorScale(rightVec, dx * scaledPan));
    targetVec = XMVectorAdd(targetVec, XMVectorScale(camUpVec, dy * scaledPan));
    XMStoreFloat3(&target, targetVec);

    // Update camera position to follow target
    UpdatePositionFromOrbit();
}

void Camera::HandleMouseZoom(float delta)
{
    // Unity-style: zoom by changing distance to target
    // Zoom speed scales with current distance for consistent feel
    float zoomAmount = delta * zoomSpeed * (distance * 0.1f);
    distance -= zoomAmount;  // Positive delta = scroll up = zoom in = reduce distance

    // Clamp distance to reasonable bounds
    if (distance < 1.0f) distance = 1.0f;
    if (distance > 500.0f) distance = 500.0f;

    // Update camera position
    UpdatePositionFromOrbit();
}

void Camera::UpdatePositionFromOrbit()
{
    float cosPitch = cosf(pitch);
    XMFLOAT3 offset;
    offset.x = sinf(yaw) * cosPitch * distance;
    offset.y = sinf(pitch) * distance;
    offset.z = cosf(yaw) * cosPitch * distance;

    XMVECTOR targetVec = XMLoadFloat3(&target);
    XMVECTOR offsetVec = XMLoadFloat3(&offset);
    XMVECTOR posVec = XMVectorAdd(targetVec, offsetVec);

    XMStoreFloat3(&position, posVec);
}

XMMATRIX Camera::GetViewMatrix() const
{
    return XMMatrixLookAtLH(
        XMLoadFloat3(&position),
        XMLoadFloat3(&target),
        XMLoadFloat3(&up)
    );
}

XMMATRIX Camera::GetProjectionMatrix() const
{
    return XMMatrixPerspectiveFovLH(fov, aspectRatio, nearPlane, farPlane);
}

//=============================================================================
// DX12Renderer Implementation
//=============================================================================

DX12Renderer::DX12Renderer()
    : frameIndex(0)
    , rtvDescriptorSize(0)
    , constantBufferData(nullptr)
    , constantBufferSize(0)
    , width(RenderConstants::kDefaultWindowWidth)
    , height(RenderConstants::kDefaultWindowHeight)
    , windowHandle(nullptr)
    , m_showVelocityVectors(false)
    , m_velocityScale(1.0f)
    , m_showCollisions(false)
    , m_collisionDisplayDuration(2.0f)
    , m_showTrajectories(false)
    , m_trajectoryUpdateInterval(0.1f)  // Sample every 100ms instead of 50ms
    , m_trajectoryMaxDuration(3.0f)  // Shorter trail duration
    , m_lastTrajectoryUpdate(0.0f)
    , m_showForces(false)
    , m_forceScale(0.01f)
    , m_forceDisplayDuration(2.0f)
    , m_heatmapMode(HeatmapMode::NONE)
    , m_heatmapMin(0.0f)
    , m_heatmapMax(20.0f)
    , m_sequentialCaptureEnabled(false)
    , m_captureInterval(1)
    , m_captureFrameCount(0)
    , m_frameCounter(0)
    , instanceBufferData(nullptr)
    , maxInstanceCount(1000)
    , m_useInstancing(true)
{
    fenceValues[0] = 0;
    fenceValues[1] = 0;
}

DX12Renderer::~DX12Renderer()
{
    Shutdown();
}

bool DX12Renderer::Initialize(HWND hwnd, UINT width, UINT height)
{
    this->width = width;
    this->height = height;
    this->windowHandle = hwnd;

    try
    {
        std::cout << "  Creating DirectX 12 device..." << std::endl;
        if (!CreateDevice()) { std::cerr << "  ERROR: CreateDevice failed!" << std::endl; return false; }
        std::cout << "  Device created successfully." << std::endl;

        std::cout << "  Creating command queue..." << std::endl;
        if (!CreateCommandQueue()) { std::cerr << "  ERROR: CreateCommandQueue failed!" << std::endl; return false; }
        std::cout << "  Command queue created successfully." << std::endl;

        std::cout << "  Creating swap chain..." << std::endl;
        if (!CreateSwapChain(hwnd, width, height)) { std::cerr << "  ERROR: CreateSwapChain failed!" << std::endl; return false; }
        std::cout << "  Swap chain created successfully." << std::endl;

        std::cout << "  Creating descriptor heaps..." << std::endl;
        if (!CreateDescriptorHeaps()) { std::cerr << "  ERROR: CreateDescriptorHeaps failed!" << std::endl; return false; }
        std::cout << "  Descriptor heaps created successfully." << std::endl;

        std::cout << "  Creating render targets..." << std::endl;
        if (!CreateRenderTargets()) { std::cerr << "  ERROR: CreateRenderTargets failed!" << std::endl; return false; }
        std::cout << "  Render targets created successfully." << std::endl;

        std::cout << "  Creating depth stencil..." << std::endl;
        if (!CreateDepthStencil()) { std::cerr << "  ERROR: CreateDepthStencil failed!" << std::endl; return false; }
        std::cout << "  Depth stencil created successfully." << std::endl;

        std::cout << "  Creating root signature..." << std::endl;
        if (!CreateRootSignature()) { std::cerr << "  ERROR: CreateRootSignature failed!" << std::endl; return false; }
        std::cout << "  Root signature created successfully." << std::endl;

        std::cout << "  Loading shaders..." << std::endl;
        if (!LoadShaders()) { std::cerr << "  ERROR: LoadShaders failed!" << std::endl; return false; }
        std::cout << "  Shaders loaded successfully." << std::endl;

        std::cout << "  Creating pipeline state..." << std::endl;
        if (!CreatePipelineState()) { std::cerr << "  ERROR: CreatePipelineState failed!" << std::endl; return false; }
        std::cout << "  Pipeline state created successfully." << std::endl;

        std::cout << "  Creating command allocators and lists..." << std::endl;
        if (!CreateCommandAllocatorsAndLists()) { std::cerr << "  ERROR: CreateCommandAllocatorsAndLists failed!" << std::endl; return false; }
        std::cout << "  Command allocators and lists created successfully." << std::endl;

        // Setup viewport
        viewport.TopLeftX = 0.0f;
        viewport.TopLeftY = 0.0f;
        viewport.Width = static_cast<float>(width);
        viewport.Height = static_cast<float>(height);
        viewport.MinDepth = 0.0f;
        viewport.MaxDepth = 1.0f;

        scissorRect.left = 0;
        scissorRect.top = 0;
        scissorRect.right = static_cast<LONG>(width);
        scissorRect.bottom = static_cast<LONG>(height);

        // Create constant buffer - allocate for kMaxObjects (supports large-scale experiments)
        const UINT singleConstantBufferSize = (sizeof(ConstantBuffer) + (RenderConstants::kConstantBufferAlignment - 1)) & ~(RenderConstants::kConstantBufferAlignment - 1);
        const UINT maxObjects = RenderConstants::kMaxObjects;
        const UINT totalConstantBufferSize = singleConstantBufferSize * maxObjects;

        ThrowIfFailed(device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
            D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(totalConstantBufferSize),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&constantBuffer)
        ));

        // Map constant buffer
        CD3DX12_RANGE readRange(0, 0);
        ThrowIfFailed(constantBuffer->Map(0, &readRange, reinterpret_cast<void**>(&constantBufferData)));

        // Store constant buffer size for later use
        constantBufferSize = singleConstantBufferSize;

        // Note: We don't create a single CBV anymore - we'll bind directly with offsets

        // Create fence for synchronization
        ThrowIfFailed(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
        fenceEvent.Reset(CreateEvent(nullptr, FALSE, FALSE, nullptr));
        if (!fenceEvent.IsValid())
        {
            ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
        }

        // Initialize camera - positioned to see the entire scene
        camera.SetPosition(0.0f, 30.0f, 60.0f);
        camera.SetLookAt(0.0f, 15.0f, 0.0f);

        // Initialize line mesh pool for visualization performance
        std::cout << "  Initializing line mesh pool..." << std::endl;
        InitializeLineMeshPool(LINE_MESH_POOL_SIZE);
        std::cout << "  Line mesh pool initialized." << std::endl;

        std::cout << "DirectX 12 initialized successfully!" << std::endl;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "DirectX 12 initialization failed: " << e.what() << std::endl;
        return false;
    }
}

void DX12Renderer::Shutdown()
{
    WaitForGPU();

    if (constantBufferData)
    {
        constantBuffer->Unmap(0, nullptr);
        constantBufferData = nullptr;
    }

    // fenceEvent is automatically closed by ScopedHandle destructor
}

bool DX12Renderer::CreateDevice()
{
    UINT dxgiFactoryFlags = 0;

#if defined(_DEBUG)
    // Enable debug layer
    ComPtr<ID3D12Debug> debugController;
    if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController))))
    {
        debugController->EnableDebugLayer();
        dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
    }
#endif

    ComPtr<IDXGIFactory4> factory;
    ThrowIfFailed(CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&factory)));

    // Try to create hardware device
    ComPtr<IDXGIAdapter1> hardwareAdapter;
    for (UINT adapterIndex = 0; factory->EnumAdapters1(adapterIndex, &hardwareAdapter) != DXGI_ERROR_NOT_FOUND; ++adapterIndex)
    {
        DXGI_ADAPTER_DESC1 desc;
        hardwareAdapter->GetDesc1(&desc);

        if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)
        {
            continue;
        }

        if (SUCCEEDED(D3D12CreateDevice(hardwareAdapter.Get(), D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&device))))
        {
            break;
        }
    }

    if (!device)
    {
        std::cerr << "Failed to create hardware device" << std::endl;
        return false;
    }

    return true;
}

bool DX12Renderer::CreateCommandQueue()
{
    D3D12_COMMAND_QUEUE_DESC queueDesc = {};
    queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
    queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

    ThrowIfFailed(device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&commandQueue)));

    return true;
}

bool DX12Renderer::CreateSwapChain(HWND hwnd, UINT width, UINT height)
{
    ComPtr<IDXGIFactory4> factory;
    ThrowIfFailed(CreateDXGIFactory2(0, IID_PPV_ARGS(&factory)));

    DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
    swapChainDesc.BufferCount = 2;
    swapChainDesc.Width = width;
    swapChainDesc.Height = height;
    swapChainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    swapChainDesc.SampleDesc.Count = 1;

    ComPtr<IDXGISwapChain1> swapChain1;
    ThrowIfFailed(factory->CreateSwapChainForHwnd(
        commandQueue.Get(),
        hwnd,
        &swapChainDesc,
        nullptr,
        nullptr,
        &swapChain1
    ));

    ThrowIfFailed(swapChain1.As(&swapChain));
    frameIndex = swapChain->GetCurrentBackBufferIndex();

    return true;
}

bool DX12Renderer::CreateDescriptorHeaps()
{
    // RTV descriptor heap
    D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
    rtvHeapDesc.NumDescriptors = 2;
    rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
    rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    ThrowIfFailed(device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&rtvHeap)));

    rtvDescriptorSize = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

    // DSV descriptor heap
    D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
    dsvHeapDesc.NumDescriptors = 1;
    dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
    dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
    ThrowIfFailed(device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&dsvHeap)));

    // CBV/SRV/UAV descriptor heap
    D3D12_DESCRIPTOR_HEAP_DESC cbvHeapDesc = {};
    cbvHeapDesc.NumDescriptors = 1;
    cbvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    cbvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(device->CreateDescriptorHeap(&cbvHeapDesc, IID_PPV_ARGS(&cbvHeap)));

    return true;
}

bool DX12Renderer::CreateRenderTargets()
{
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap->GetCPUDescriptorHandleForHeapStart());

    for (UINT i = 0; i < 2; i++)
    {
        ThrowIfFailed(swapChain->GetBuffer(i, IID_PPV_ARGS(&renderTargets[i])));
        device->CreateRenderTargetView(renderTargets[i].Get(), nullptr, rtvHandle);
        rtvHandle.Offset(1, rtvDescriptorSize);
    }

    return true;
}

bool DX12Renderer::CreateDepthStencil()
{
    D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilDesc = {};
    depthStencilDesc.Format = DXGI_FORMAT_D32_FLOAT;
    depthStencilDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    depthStencilDesc.Flags = D3D12_DSV_FLAG_NONE;

    D3D12_CLEAR_VALUE depthOptimizedClearValue = {};
    depthOptimizedClearValue.Format = DXGI_FORMAT_D32_FLOAT;
    depthOptimizedClearValue.DepthStencil.Depth = 1.0f;
    depthOptimizedClearValue.DepthStencil.Stencil = 0;

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Tex2D(DXGI_FORMAT_D32_FLOAT, width, height, 1, 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL),
        D3D12_RESOURCE_STATE_DEPTH_WRITE,
        &depthOptimizedClearValue,
        IID_PPV_ARGS(&depthStencil)
    ));

    device->CreateDepthStencilView(depthStencil.Get(), &depthStencilDesc, dsvHeap->GetCPUDescriptorHandleForHeapStart());

    return true;
}

// Continued in next part...
// This file contains the continuation of DX12Renderer.cpp
// Merge this content into DX12Renderer.cpp after the "// Continued in next part..." comment

bool DX12Renderer::CreateRootSignature()
{
    // Root parameter for constant buffer
    CD3DX12_ROOT_PARAMETER1 rootParameters[1];
    rootParameters[0].InitAsConstantBufferView(0, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_ALL);

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDesc;
    rootSignatureDesc.Init_1_1(_countof(rootParameters), rootParameters, 0, nullptr, D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    ComPtr<ID3DBlob> signature;
    ComPtr<ID3DBlob> error;
    ThrowIfFailed(D3DX12SerializeVersionedRootSignature(&rootSignatureDesc, D3D_ROOT_SIGNATURE_VERSION_1, &signature, &error));
    ThrowIfFailed(device->CreateRootSignature(0, signature->GetBufferPointer(), signature->GetBufferSize(), IID_PPV_ARGS(&rootSignature)));

    return true;
}

bool DX12Renderer::LoadShaders()
{
    UINT compileFlags = D3DCOMPILE_DEBUG | D3DCOMPILE_SKIP_OPTIMIZATION;

    // Get shader path
    wchar_t shaderPath[MAX_PATH];
    GetModuleFileNameW(nullptr, shaderPath, MAX_PATH);
    std::wstring exePath(shaderPath);
    size_t lastSlash = exePath.find_last_of(L"\\/");
    std::wstring shaderDir = exePath.substr(0, lastSlash) + L"\\..\\..\\shaders\\";

    std::wstring vsPath = shaderDir + L"PhysicsVS.hlsl";
    std::wstring psPath = shaderDir + L"PhysicsPS.hlsl";

    // Print shader paths for debugging
    std::wcout << L"    Vertex Shader Path: " << vsPath << std::endl;
    std::wcout << L"    Pixel Shader Path: " << psPath << std::endl;

    ComPtr<ID3DBlob> error;

    // Compile vertex shader
    std::cout << "    Compiling vertex shader..." << std::endl;
    HRESULT hr = D3DCompileFromFile(
        vsPath.c_str(),
        nullptr,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,
        "main",
        "vs_5_0",
        compileFlags,
        0,
        &vertexShader,
        &error
    );

    if (FAILED(hr))
    {
        std::cerr << "    ERROR: Vertex shader compilation failed! HRESULT: 0x" << std::hex << hr << std::dec << std::endl;
        if (error)
        {
            std::cerr << "    Vertex shader compilation error: " << (char*)error->GetBufferPointer() << std::endl;
        }
        else
        {
            std::cerr << "    No error details available. File may not exist at path above." << std::endl;
        }
        return false;
    }
    std::cout << "    Vertex shader compiled successfully." << std::endl;

    // Compile pixel shader
    std::cout << "    Compiling pixel shader..." << std::endl;
    hr = D3DCompileFromFile(
        psPath.c_str(),
        nullptr,
        D3D_COMPILE_STANDARD_FILE_INCLUDE,
        "main",
        "ps_5_0",
        compileFlags,
        0,
        &pixelShader,
        &error
    );

    if (FAILED(hr))
    {
        std::cerr << "    ERROR: Pixel shader compilation failed! HRESULT: 0x" << std::hex << hr << std::dec << std::endl;
        if (error)
        {
            std::cerr << "    Pixel shader compilation error: " << (char*)error->GetBufferPointer() << std::endl;
        }
        else
        {
            std::cerr << "    No error details available. File may not exist at path above." << std::endl;
        }
        return false;
    }
    std::cout << "    Pixel shader compiled successfully." << std::endl;

    return true;
}

bool DX12Renderer::CreatePipelineState()
{
    // Input layout
    D3D12_INPUT_ELEMENT_DESC inputElementDescs[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
    };

    // Pipeline state object description
    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = { inputElementDescs, _countof(inputElementDescs) };
    psoDesc.pRootSignature = rootSignature.Get();
    psoDesc.VS = CD3DX12_SHADER_BYTECODE(vertexShader.Get());
    psoDesc.PS = CD3DX12_SHADER_BYTECODE(pixelShader.Get());
    psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    psoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_BACK;
    psoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
    psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    psoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
    psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;
    psoDesc.SampleDesc.Count = 1;

    ThrowIfFailed(device->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&pipelineState)));

    return true;
}

bool DX12Renderer::CreateCommandAllocatorsAndLists()
{
    for (UINT i = 0; i < 2; i++)
    {
        ThrowIfFailed(device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&commandAllocators[i])));
    }

    ThrowIfFailed(device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, commandAllocators[0].Get(), pipelineState.Get(), IID_PPV_ARGS(&commandList)));
    ThrowIfFailed(commandList->Close());

    return true;
}

void DX12Renderer::BeginFrame()
{
    // Note: Mesh caching system now uses persistent meshCache (std::unordered_map)
    // No need to clear dynamicMeshCache as we're reusing cached meshes across frames

    // Reset line mesh pool for this frame's visualization
    ResetLineMeshPool();

    // Reset command allocator and list
    ThrowIfFailed(commandAllocators[frameIndex]->Reset());
    ThrowIfFailed(commandList->Reset(commandAllocators[frameIndex].Get(), pipelineState.Get()));

    // Set necessary state
    commandList->SetGraphicsRootSignature(rootSignature.Get());

    ID3D12DescriptorHeap* ppHeaps[] = { cbvHeap.Get() };
    commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);
    // Note: Constant buffer is now bound per-object in RenderActor() with appropriate offset

    commandList->RSSetViewports(1, &viewport);
    commandList->RSSetScissorRects(1, &scissorRect);

    // Transition render target to render target state
    commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTargets[frameIndex].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

    // Get render target and depth stencil handles
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap->GetCPUDescriptorHandleForHeapStart(), frameIndex, rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(dsvHeap->GetCPUDescriptorHandleForHeapStart());

    commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

    // Clear render target and depth stencil
    const float clearColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };  // White background
    commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);

    commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
}

void DX12Renderer::EndFrame()
{
    // Transition render target to present state
    commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTargets[frameIndex].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));

    ThrowIfFailed(commandList->Close());

    // Execute command list
    ID3D12CommandList* ppCommandLists[] = { commandList.Get() };
    commandQueue->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);

    // Present
    ThrowIfFailed(swapChain->Present(1, 0));

    MoveToNextFrame();
}

void DX12Renderer::WaitForGPU()
{
    if (commandQueue && fence)
    {
        const UINT64 fenceValue = fenceValues[frameIndex];
        ThrowIfFailed(commandQueue->Signal(fence.Get(), fenceValue));

        if (fence->GetCompletedValue() < fenceValue)
        {
            ThrowIfFailed(fence->SetEventOnCompletion(fenceValue, fenceEvent.Get()));
            WaitForSingleObject(fenceEvent.Get(), INFINITE);
        }

        fenceValues[frameIndex]++;
    }
}

void DX12Renderer::MoveToNextFrame()
{
    const UINT64 currentFenceValue = fenceValues[frameIndex];
    ThrowIfFailed(commandQueue->Signal(fence.Get(), currentFenceValue));

    frameIndex = swapChain->GetCurrentBackBufferIndex();

    if (fence->GetCompletedValue() < fenceValues[frameIndex])
    {
        ThrowIfFailed(fence->SetEventOnCompletion(fenceValues[frameIndex], fenceEvent.Get()));
        WaitForSingleObject(fenceEvent.Get(), INFINITE);
    }

    fenceValues[frameIndex] = currentFenceValue + 1;
}

void DX12Renderer::UpdateConstantBuffer(const XMMATRIX& world, UINT objectIndex)
{
    ConstantBuffer cb;

    XMMATRIX view = camera.GetViewMatrix();
    XMMATRIX proj = camera.GetProjectionMatrix();

    cb.worldViewProj = XMMatrixTranspose(world * view * proj);
    cb.world = XMMatrixTranspose(world);
    cb.lightDir = XMFLOAT4(0.3f, -0.7f, 0.3f, 0.0f);
    cb.cameraPos = XMFLOAT4(camera.GetPosition().x, camera.GetPosition().y, camera.GetPosition().z, 1.0f);

    // Write to the specific offset in the constant buffer
    UINT8* destination = constantBufferData + (objectIndex * constantBufferSize);
    memcpy(destination, &cb, sizeof(cb));
}

void DX12Renderer::SetCamera(const Camera& cam)
{
    camera = cam;
}

void DX12Renderer::Resize(UINT newWidth, UINT newHeight)
{
    if (newWidth == 0 || newHeight == 0)
        return;

    if (newWidth == this->width && newHeight == this->height)
        return;

    // Wait for GPU to finish all work
    WaitForGPU();

    // Release references to render target views
    for (UINT i = 0; i < 2; i++)
    {
        renderTargets[i].Reset();
        fenceValues[i] = fenceValues[frameIndex];
    }

    // Release depth stencil
    depthStencil.Reset();

    // Resize swap chain buffers
    DXGI_SWAP_CHAIN_DESC desc = {};
    swapChain->GetDesc(&desc);
    HRESULT hr = swapChain->ResizeBuffers(2, newWidth, newHeight, desc.BufferDesc.Format, desc.Flags);
    if (FAILED(hr))
    {
        std::cerr << "Failed to resize swap chain buffers" << std::endl;
        return;
    }

    frameIndex = swapChain->GetCurrentBackBufferIndex();

    // Update dimensions
    this->width = newWidth;
    this->height = newHeight;

    // Recreate render targets
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap->GetCPUDescriptorHandleForHeapStart());
    for (UINT i = 0; i < 2; i++)
    {
        ThrowIfFailed(swapChain->GetBuffer(i, IID_PPV_ARGS(&renderTargets[i])));
        device->CreateRenderTargetView(renderTargets[i].Get(), nullptr, rtvHandle);
        rtvHandle.Offset(1, rtvDescriptorSize);
    }

    // Recreate depth stencil
    D3D12_RESOURCE_DESC depthDesc = {};
    depthDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    depthDesc.Width = newWidth;
    depthDesc.Height = newHeight;
    depthDesc.DepthOrArraySize = 1;
    depthDesc.MipLevels = 1;
    depthDesc.Format = DXGI_FORMAT_D32_FLOAT;
    depthDesc.SampleDesc.Count = 1;
    depthDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

    D3D12_CLEAR_VALUE clearValue = {};
    clearValue.Format = DXGI_FORMAT_D32_FLOAT;
    clearValue.DepthStencil.Depth = 1.0f;

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
        D3D12_HEAP_FLAG_NONE,
        &depthDesc,
        D3D12_RESOURCE_STATE_DEPTH_WRITE,
        &clearValue,
        IID_PPV_ARGS(&depthStencil)
    ));

    D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
    dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    device->CreateDepthStencilView(depthStencil.Get(), &dsvDesc, dsvHeap->GetCPUDescriptorHandleForHeapStart());

    // Update viewport and scissor rect
    viewport.Width = static_cast<float>(newWidth);
    viewport.Height = static_cast<float>(newHeight);
    viewport.MinDepth = 0.0f;
    viewport.MaxDepth = 1.0f;
    viewport.TopLeftX = 0.0f;
    viewport.TopLeftY = 0.0f;

    scissorRect.left = 0;
    scissorRect.top = 0;
    scissorRect.right = static_cast<LONG>(newWidth);
    scissorRect.bottom = static_cast<LONG>(newHeight);

    // Update camera aspect ratio
    camera.SetAspectRatio(static_cast<float>(newWidth) / static_cast<float>(newHeight));
}

// Continued in Part 3...
// This file contains the final part of DX12Renderer.cpp
// Merge this content after Part 2

void DX12Renderer::RenderPhysXScene(PxScene* scene)
{
    if (!scene) return;

    // Get all actors
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    // Track object index for constant buffer offsets
    UINT objectIndex = 0;

    // Render each actor
    for (PxU32 i = 0; i < nbActors; i++)
    {
        PxRigidActor* actor = actors[i];

        // Determine color based on actor type
        XMFLOAT3 color(0.5f, 0.5f, 0.5f);
        if (actor->is<PxRigidDynamic>())
        {
            color = XMFLOAT3(0.3f, 0.7f, 0.3f); // Green for dynamic
        }
        else if (actor->is<PxRigidStatic>())
        {
            color = XMFLOAT3(0.6f, 0.6f, 0.6f); // Gray for static
        }

        RenderActor(actor, color, objectIndex);
    }

    // Render visualization overlays
    if (m_showVelocityVectors)
    {
        RenderVelocityVectors(scene, objectIndex);
    }

    if (m_showCollisions)
    {
        RenderCollisionPoints(objectIndex);
    }

    if (m_showTrajectories)
    {
        RenderTrajectories(objectIndex);
    }

    if (m_showForces)
    {
        RenderForces(scene, objectIndex);
    }

    if (m_showJoints)
    {
        RenderJoints(scene, objectIndex);
    }

    // Render break effects
    RenderBreakEffects(objectIndex);

    // Render deformable volumes (soft bodies)
    if (gDeformableManager)
    {
        RenderDeformableVolumes(gDeformableManager.get(), objectIndex);
    }
}

void DX12Renderer::RenderActor(PxRigidActor* actor, const XMFLOAT3& color, UINT& objectIndex)
{
    if (!actor) return;

    // Override color with heatmap color if heatmap mode is active
    XMFLOAT3 renderColor = color;
    if (m_heatmapMode != HeatmapMode::NONE)
    {
        renderColor = GetHeatmapColorForActor(actor);
    }

    PxU32 nShapes = actor->getNbShapes();
    if (nShapes == 0) return;

    std::vector<PxShape*> shapes(nShapes);
    actor->getShapes(shapes.data(), nShapes);

    for (PxU32 i = 0; i < nShapes; i++)
    {
        PxShape* shape = shapes[i];
        if (!shape) continue;

        // Get transform
        PxTransform pxTransform = PxShapeExt::getGlobalPose(*shape, *actor);
        PxMat44 pxMat(pxTransform);

        // Convert PhysX matrix to DirectXMath
        XMMATRIX world = XMMatrixSet(
            pxMat[0][0], pxMat[0][1], pxMat[0][2], pxMat[0][3],
            pxMat[1][0], pxMat[1][1], pxMat[1][2], pxMat[1][3],
            pxMat[2][0], pxMat[2][1], pxMat[2][2], pxMat[2][3],
            pxMat[3][0], pxMat[3][1], pxMat[3][2], pxMat[3][3]
        );

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            static bool warningShown = false;
            if (!warningShown)
            {
                std::cerr << "WARNING: Too many objects (" << objectIndex << ") - exceeding constant buffer capacity. Skipping remaining objects." << std::endl;
                warningShown = true;
            }
            return;  // Skip rendering this and remaining objects
        }

        // Update constant buffer at the specific offset for this object
        UpdateConstantBuffer(world, objectIndex);

        // CRITICAL FIX: Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        // Get geometry using PxGeometryHolder
        PxGeometryHolder geomHolder = shape->getGeometry();
        PxGeometryType::Enum geomType = geomHolder.getType();

        switch (geomType)
        {
        case PxGeometryType::eBOX:
        {
            const PxBoxGeometry& boxGeom = geomHolder.box();
            auto meshPtr = GetOrCreateBoxMesh(boxGeom, renderColor);

            commandList->IASetVertexBuffers(0, 1, &meshPtr->vertexBufferView);
            commandList->IASetIndexBuffer(&meshPtr->indexBufferView);
            commandList->DrawIndexedInstanced(meshPtr->indexCount, 1, 0, 0, 0);

            objectIndex++;  // Increment for next object
            break;
        }
        case PxGeometryType::eSPHERE:
        {
            const PxSphereGeometry& sphereGeom = geomHolder.sphere();
            auto meshPtr = GetOrCreateSphereMesh(sphereGeom, renderColor);

            commandList->IASetVertexBuffers(0, 1, &meshPtr->vertexBufferView);
            commandList->IASetIndexBuffer(&meshPtr->indexBufferView);
            commandList->DrawIndexedInstanced(meshPtr->indexCount, 1, 0, 0, 0);

            objectIndex++;  // Increment for next object
            break;
        }
        case PxGeometryType::eCAPSULE:
        {
            const PxCapsuleGeometry& capsuleGeom = geomHolder.capsule();
            RenderMesh mesh = CreateCapsuleMesh(capsuleGeom, renderColor);

            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);

            objectIndex++;
            break;
        }
        case PxGeometryType::eCONVEXMESH:
        {
            const PxConvexMeshGeometry& convexGeom = geomHolder.convexMesh();
            RenderMesh mesh = CreateConvexMesh(convexGeom, renderColor);

            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);

            objectIndex++;
            break;
        }
        case PxGeometryType::eTRIANGLEMESH:
        {
            const PxTriangleMeshGeometry& triGeom = geomHolder.triangleMesh();
            RenderMesh mesh = CreateTriangleMesh(triGeom, renderColor);

            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);

            objectIndex++;
            break;
        }
        case PxGeometryType::eHEIGHTFIELD:
        {
            const PxHeightFieldGeometry& hfGeom = geomHolder.heightField();
            RenderMesh mesh = CreateHeightfieldMesh(hfGeom, renderColor);

            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);

            objectIndex++;
            break;
        }
        case PxGeometryType::ePLANE:
        {
            // Skip rendering infinite planes - we have a visible ground box instead
            // Infinite planes don't have proper transforms and cause rendering issues
            break;
        }
        default:
            // Unsupported geometry type
            break;
        }
    }
}

//=============================================================================
// Mesh Caching System
//=============================================================================

std::string DX12Renderer::GenerateBoxCacheKey(const PxBoxGeometry& geometry)
{
    char key[128];
    snprintf(key, sizeof(key), "box_%.3f_%.3f_%.3f",
             geometry.halfExtents.x, geometry.halfExtents.y, geometry.halfExtents.z);
    return std::string(key);
}

std::string DX12Renderer::GenerateSphereCacheKey(const PxSphereGeometry& geometry)
{
    char key[128];
    snprintf(key, sizeof(key), "sphere_%.3f", geometry.radius);
    return std::string(key);
}

std::shared_ptr<RenderMesh> DX12Renderer::GetOrCreateBoxMesh(const PxBoxGeometry& geometry, const XMFLOAT3& color)
{
    std::string cacheKey = GenerateBoxCacheKey(geometry);

    auto it = meshCache.find(cacheKey);
    if (it != meshCache.end())
    {
        // Mesh found in cache - reuse it
        return it->second;
    }

    // Mesh not in cache - create new one
    auto meshPtr = std::make_shared<RenderMesh>(CreateBoxMesh(geometry, color));
    meshCache[cacheKey] = meshPtr;

    // std::cout << "Created and cached new box mesh: " << cacheKey << " (cache size: " << meshCache.size() << ")" << std::endl;

    return meshPtr;
}

std::shared_ptr<RenderMesh> DX12Renderer::GetOrCreateSphereMesh(const PxSphereGeometry& geometry, const XMFLOAT3& color)
{
    std::string cacheKey = GenerateSphereCacheKey(geometry);

    auto it = meshCache.find(cacheKey);
    if (it != meshCache.end())
    {
        // Mesh found in cache - reuse it
        return it->second;
    }

    // Mesh not in cache - create new one
    auto meshPtr = std::make_shared<RenderMesh>(CreateSphereMesh(geometry, color));
    meshCache[cacheKey] = meshPtr;

    // std::cout << "Created and cached new sphere mesh: " << cacheKey << " (cache size: " << meshCache.size() << ")" << std::endl;

    return meshPtr;
}

RenderMesh DX12Renderer::CreateBoxMesh(const PxBoxGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    PxVec3 halfExtents = geometry.halfExtents;

    // Box vertices (8 corners)
    Vertex vertices[] =
    {
        // Front face
        { XMFLOAT3(-halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 0, 1), color },
        // Back face
        { XMFLOAT3( halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        { XMFLOAT3(-halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 0, -1), color },
        // Top face
        { XMFLOAT3(-halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(0, 1, 0), color },
        // Bottom face
        { XMFLOAT3(-halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, -1, 0), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(0, -1, 0), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, -1, 0), color },
        { XMFLOAT3(-halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(0, -1, 0), color },
        // Right face
        { XMFLOAT3( halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(1, 0, 0), color },
        { XMFLOAT3( halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(1, 0, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(1, 0, 0), color },
        { XMFLOAT3( halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(1, 0, 0), color },
        // Left face
        { XMFLOAT3(-halfExtents.x, -halfExtents.y, -halfExtents.z), XMFLOAT3(-1, 0, 0), color },
        { XMFLOAT3(-halfExtents.x, -halfExtents.y,  halfExtents.z), XMFLOAT3(-1, 0, 0), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y,  halfExtents.z), XMFLOAT3(-1, 0, 0), color },
        { XMFLOAT3(-halfExtents.x,  halfExtents.y, -halfExtents.z), XMFLOAT3(-1, 0, 0), color },
    };

    UINT indices[] =
    {
        0, 1, 2,  0, 2, 3,    // Front
        4, 5, 6,  4, 6, 7,    // Back
        8, 9, 10, 8, 10, 11,  // Top
        12, 13, 14, 12, 14, 15, // Bottom
        16, 17, 18, 16, 18, 19, // Right
        20, 21, 22, 20, 22, 23  // Left
    };

    mesh.indexCount = _countof(indices);

    // Create vertex buffer
    const UINT vertexBufferSize = sizeof(vertices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices, vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = sizeof(indices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices, indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreateSphereMesh(const PxSphereGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const int segments = RenderConstants::kSphereSegments;
    const int rings = RenderConstants::kSphereRings;
    const float radius = geometry.radius;

    std::vector<Vertex> vertices;
    std::vector<UINT> indices;

    // Generate sphere vertices
    for (int ring = 0; ring <= rings; ring++)
    {
        float phi = XM_PI * ring / rings;
        float y = radius * cosf(phi);
        float ringRadius = radius * sinf(phi);

        for (int segment = 0; segment <= segments; segment++)
        {
            float theta = 2.0f * XM_PI * segment / segments;
            float x = ringRadius * cosf(theta);
            float z = ringRadius * sinf(theta);

            XMFLOAT3 position(x, y, z);
            XMFLOAT3 normal = position;
            XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&normal)));

            vertices.push_back({ position, normal, color });
        }
    }

    // Generate sphere indices
    for (int ring = 0; ring < rings; ring++)
    {
        for (int segment = 0; segment < segments; segment++)
        {
            int current = ring * (segments + 1) + segment;
            int next = current + segments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    mesh.indexCount = static_cast<UINT>(indices.size());

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreateCapsuleMesh(const PxCapsuleGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const int segments = RenderConstants::kSphereSegments;
    const int rings = RenderConstants::kSphereRings / 2;  // Half rings for each hemisphere
    const float radius = geometry.radius;
    const float halfHeight = geometry.halfHeight;

    std::vector<Vertex> vertices;
    std::vector<UINT> indices;

    // PhysX capsule is aligned along X axis
    // Generate top hemisphere (positive X direction)
    for (int ring = 0; ring <= rings; ring++)
    {
        float phi = (XM_PI / 2.0f) * ring / rings;  // 0 to PI/2
        float localX = radius * cosf(phi);
        float ringRadius = radius * sinf(phi);

        for (int segment = 0; segment <= segments; segment++)
        {
            float theta = 2.0f * XM_PI * segment / segments;
            float y = ringRadius * cosf(theta);
            float z = ringRadius * sinf(theta);
            float x = halfHeight + localX;  // Offset by halfHeight

            XMFLOAT3 position(x, y, z);
            XMFLOAT3 normal(localX / radius, y / radius, z / radius);
            XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&normal)));

            vertices.push_back({ position, normal, color });
        }
    }

    // Generate cylinder body
    int cylinderStartIdx = static_cast<int>(vertices.size());
    const int cylinderRings = 2;  // Just need 2 rings for the cylinder

    for (int ring = 0; ring <= cylinderRings; ring++)
    {
        float x = halfHeight - (2.0f * halfHeight * ring / cylinderRings);

        for (int segment = 0; segment <= segments; segment++)
        {
            float theta = 2.0f * XM_PI * segment / segments;
            float y = radius * cosf(theta);
            float z = radius * sinf(theta);

            XMFLOAT3 position(x, y, z);
            XMFLOAT3 normal(0.0f, y / radius, z / radius);
            XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&normal)));

            vertices.push_back({ position, normal, color });
        }
    }

    // Generate bottom hemisphere (negative X direction)
    int bottomStartIdx = static_cast<int>(vertices.size());
    for (int ring = 0; ring <= rings; ring++)
    {
        float phi = (XM_PI / 2.0f) + (XM_PI / 2.0f) * ring / rings;  // PI/2 to PI
        float localX = radius * cosf(phi);
        float ringRadius = radius * sinf(phi);

        for (int segment = 0; segment <= segments; segment++)
        {
            float theta = 2.0f * XM_PI * segment / segments;
            float y = ringRadius * cosf(theta);
            float z = ringRadius * sinf(theta);
            float x = -halfHeight + localX;  // Offset by -halfHeight

            XMFLOAT3 position(x, y, z);
            XMFLOAT3 normal(localX / radius, y / radius, z / radius);
            XMStoreFloat3(&normal, XMVector3Normalize(XMLoadFloat3(&normal)));

            vertices.push_back({ position, normal, color });
        }
    }

    // Generate indices for top hemisphere
    for (int ring = 0; ring < rings; ring++)
    {
        for (int segment = 0; segment < segments; segment++)
        {
            int current = ring * (segments + 1) + segment;
            int next = current + segments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    // Generate indices for cylinder
    for (int ring = 0; ring < cylinderRings; ring++)
    {
        for (int segment = 0; segment < segments; segment++)
        {
            int current = cylinderStartIdx + ring * (segments + 1) + segment;
            int next = current + segments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    // Generate indices for bottom hemisphere
    for (int ring = 0; ring < rings; ring++)
    {
        for (int segment = 0; segment < segments; segment++)
        {
            int current = bottomStartIdx + ring * (segments + 1) + segment;
            int next = current + segments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    mesh.indexCount = static_cast<UINT>(indices.size());

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreateConvexMesh(const PxConvexMeshGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const PxConvexMesh* convexMesh = geometry.convexMesh;
    if (!convexMesh)
    {
        // Return empty mesh if no convex mesh available
        mesh.indexCount = 0;
        return mesh;
    }

    // Get vertex data from convex mesh
    PxU32 nbVerts = convexMesh->getNbVertices();
    const PxVec3* verts = convexMesh->getVertices();

    // Get polygon data
    PxU32 nbPolygons = convexMesh->getNbPolygons();
    const PxU8* indexBuffer = convexMesh->getIndexBuffer();

    // Build vertices and indices for rendering
    std::vector<Vertex> vertices;
    std::vector<UINT> indices;

    // For each polygon, triangulate it (fan triangulation)
    for (PxU32 p = 0; p < nbPolygons; p++)
    {
        PxHullPolygon polygon;
        convexMesh->getPolygonData(p, polygon);

        // Calculate face normal
        PxVec3 normal(polygon.mPlane[0], polygon.mPlane[1], polygon.mPlane[2]);
        normal.normalize();
        XMFLOAT3 normalF(normal.x, normal.y, normal.z);

        // Get vertex indices for this polygon
        UINT baseVertex = static_cast<UINT>(vertices.size());

        // Add vertices for this face
        for (PxU32 v = 0; v < polygon.mNbVerts; v++)
        {
            PxU8 vertIndex = indexBuffer[polygon.mIndexBase + v];
            // Apply scale component-wise
            PxVec3 pos = verts[vertIndex].multiply(geometry.scale.scale);

            Vertex vert;
            vert.position = XMFLOAT3(pos.x, pos.y, pos.z);
            vert.normal = normalF;
            vert.color = color;
            vertices.push_back(vert);
        }

        // Triangulate (fan from first vertex)
        for (PxU32 v = 1; v < polygon.mNbVerts - 1; v++)
        {
            indices.push_back(baseVertex);
            indices.push_back(baseVertex + v);
            indices.push_back(baseVertex + v + 1);
        }
    }

    mesh.indexCount = static_cast<UINT>(indices.size());

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));

    CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_UPLOAD);
    CD3DX12_RESOURCE_DESC vertexBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &vertexBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)));

    D3D12_RANGE readRange = {};
    UINT8* pVertexDataBegin;
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));
    CD3DX12_RESOURCE_DESC indexBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &indexBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreateTriangleMesh(const PxTriangleMeshGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const PxTriangleMesh* triMesh = geometry.triangleMesh;
    if (!triMesh)
    {
        mesh.indexCount = 0;
        return mesh;
    }

    // Get vertex and triangle data
    PxU32 nbVerts = triMesh->getNbVertices();
    PxU32 nbTris = triMesh->getNbTriangles();
    const PxVec3* verts = triMesh->getVertices();

    // Build render vertices with per-face normals
    std::vector<Vertex> vertices;
    std::vector<UINT> indices;

    // Check if using 16-bit or 32-bit indices
    bool has16BitIndices = triMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;

    for (PxU32 t = 0; t < nbTris; t++)
    {
        PxU32 i0, i1, i2;

        if (has16BitIndices)
        {
            const PxU16* tris = reinterpret_cast<const PxU16*>(triMesh->getTriangles());
            i0 = tris[t * 3 + 0];
            i1 = tris[t * 3 + 1];
            i2 = tris[t * 3 + 2];
        }
        else
        {
            const PxU32* tris = reinterpret_cast<const PxU32*>(triMesh->getTriangles());
            i0 = tris[t * 3 + 0];
            i1 = tris[t * 3 + 1];
            i2 = tris[t * 3 + 2];
        }

        // Apply scale (component-wise multiplication)
        PxVec3 v0 = verts[i0].multiply(geometry.scale.scale);
        PxVec3 v1 = verts[i1].multiply(geometry.scale.scale);
        PxVec3 v2 = verts[i2].multiply(geometry.scale.scale);

        // Calculate face normal
        PxVec3 edge1 = v1 - v0;
        PxVec3 edge2 = v2 - v0;
        PxVec3 normal = edge1.cross(edge2);
        normal.normalize();
        XMFLOAT3 normalF(normal.x, normal.y, normal.z);

        UINT baseIndex = static_cast<UINT>(vertices.size());

        Vertex vert0 = { XMFLOAT3(v0.x, v0.y, v0.z), normalF, color };
        Vertex vert1 = { XMFLOAT3(v1.x, v1.y, v1.z), normalF, color };
        Vertex vert2 = { XMFLOAT3(v2.x, v2.y, v2.z), normalF, color };

        vertices.push_back(vert0);
        vertices.push_back(vert1);
        vertices.push_back(vert2);

        indices.push_back(baseIndex);
        indices.push_back(baseIndex + 1);
        indices.push_back(baseIndex + 2);
    }

    mesh.indexCount = static_cast<UINT>(indices.size());

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));

    CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_UPLOAD);
    CD3DX12_RESOURCE_DESC vertexBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &vertexBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)));

    D3D12_RANGE readRange = {};
    UINT8* pVertexDataBegin;
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));
    CD3DX12_RESOURCE_DESC indexBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &indexBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreateHeightfieldMesh(const PxHeightFieldGeometry& geometry, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const PxHeightField* heightField = geometry.heightField;
    if (!heightField)
    {
        mesh.indexCount = 0;
        return mesh;
    }

    PxU32 nbRows = heightField->getNbRows();
    PxU32 nbCols = heightField->getNbColumns();

    std::vector<Vertex> vertices;
    std::vector<UINT> indices;

    // Generate vertices from heightfield samples
    for (PxU32 r = 0; r < nbRows; r++)
    {
        for (PxU32 c = 0; c < nbCols; c++)
        {
            // Get height sample
            PxReal height = heightField->getHeight(static_cast<PxReal>(r), static_cast<PxReal>(c));

            // Apply scale
            float x = static_cast<float>(r) * geometry.rowScale;
            float y = height * geometry.heightScale;
            float z = static_cast<float>(c) * geometry.columnScale;

            // Calculate normal (approximate using neighboring heights)
            PxReal h01 = (r > 0) ? heightField->getHeight(static_cast<PxReal>(r - 1), static_cast<PxReal>(c)) : height;
            PxReal h21 = (r < nbRows - 1) ? heightField->getHeight(static_cast<PxReal>(r + 1), static_cast<PxReal>(c)) : height;
            PxReal h10 = (c > 0) ? heightField->getHeight(static_cast<PxReal>(r), static_cast<PxReal>(c - 1)) : height;
            PxReal h12 = (c < nbCols - 1) ? heightField->getHeight(static_cast<PxReal>(r), static_cast<PxReal>(c + 1)) : height;

            float dx = (h21 - h01) * geometry.heightScale / (2.0f * geometry.rowScale);
            float dz = (h12 - h10) * geometry.heightScale / (2.0f * geometry.columnScale);

            XMFLOAT3 normal(-dx, 1.0f, -dz);
            // Normalize
            float len = sqrtf(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            if (len > 0)
            {
                normal.x /= len;
                normal.y /= len;
                normal.z /= len;
            }

            Vertex vert;
            vert.position = XMFLOAT3(x, y, z);
            vert.normal = normal;
            vert.color = color;
            vertices.push_back(vert);
        }
    }

    // Generate indices for triangles
    for (PxU32 r = 0; r < nbRows - 1; r++)
    {
        for (PxU32 c = 0; c < nbCols - 1; c++)
        {
            UINT topLeft = r * nbCols + c;
            UINT topRight = topLeft + 1;
            UINT bottomLeft = (r + 1) * nbCols + c;
            UINT bottomRight = bottomLeft + 1;

            // First triangle
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            // Second triangle
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    mesh.indexCount = static_cast<UINT>(indices.size());

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));

    CD3DX12_HEAP_PROPERTIES heapProps(D3D12_HEAP_TYPE_UPLOAD);
    CD3DX12_RESOURCE_DESC vertexBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &vertexBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)));

    D3D12_RANGE readRange = {};
    UINT8* pVertexDataBegin;
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));
    CD3DX12_RESOURCE_DESC indexBufferDesc = CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize);

    ThrowIfFailed(device->CreateCommittedResource(
        &heapProps,
        D3D12_HEAP_FLAG_NONE,
        &indexBufferDesc,
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

RenderMesh DX12Renderer::CreatePlaneMesh(const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    const float size = 50.0f;

    Vertex vertices[] =
    {
        { XMFLOAT3(-size, 0, -size), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( size, 0, -size), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3( size, 0,  size), XMFLOAT3(0, 1, 0), color },
        { XMFLOAT3(-size, 0,  size), XMFLOAT3(0, 1, 0), color },
    };

    UINT indices[] = { 0, 1, 2, 0, 2, 3 };

    mesh.indexCount = _countof(indices);

    // Create vertex buffer
    const UINT vertexBufferSize = sizeof(vertices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices, vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = sizeof(indices);
    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices, indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    return mesh;
}

//=============================================================================
// ImGui Integration
//=============================================================================

void DX12Renderer::ApplyModernStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();

    // Enable anti-aliased lines and fill
    style.AntiAliasedLines = true;
    style.AntiAliasedFill = true;

    // Rounding for modern look
    style.WindowRounding = 8.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 4.0f;

    // Spacing and padding
    style.WindowPadding = ImVec2(12.0f, 12.0f);
    style.FramePadding = ImVec2(8.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
    style.IndentSpacing = 20.0f;
    style.ScrollbarSize = 14.0f;
    style.GrabMinSize = 12.0f;

    // Border sizes
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.TabBorderSize = 0.0f;

    // Colors - Professional dark theme with blue accent
    ImVec4* colors = style.Colors;

    // Background colors
    colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.12f, 0.95f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.08f, 0.08f, 0.10f, 0.90f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.12f, 0.12f, 0.14f, 0.98f);

    // Borders
    colors[ImGuiCol_Border] = ImVec4(0.30f, 0.30f, 0.35f, 0.50f);
    colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

    // Frame backgrounds
    colors[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.22f, 0.26f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);

    // Title bars
    colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.08f, 0.10f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.14f, 0.18f, 0.22f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.08f, 0.08f, 0.10f, 0.75f);

    // Menu bar
    colors[ImGuiCol_MenuBarBg] = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);

    // Scrollbar
    colors[ImGuiCol_ScrollbarBg] = ImVec4(0.08f, 0.08f, 0.10f, 0.60f);
    colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.40f, 0.40f, 0.45f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.50f, 0.50f, 0.55f, 1.00f);

    // Check mark
    colors[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);

    // Slider
    colors[ImGuiCol_SliderGrab] = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.36f, 0.66f, 1.00f, 1.00f);

    // Button
    colors[ImGuiCol_Button] = ImVec4(0.20f, 0.40f, 0.60f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.26f, 0.50f, 0.74f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.30f, 0.56f, 0.82f, 1.00f);

    // Header (collapsible sections, tree nodes)
    colors[ImGuiCol_Header] = ImVec4(0.20f, 0.40f, 0.60f, 0.70f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.50f, 0.74f, 0.80f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.56f, 0.82f, 1.00f);

    // Separator
    colors[ImGuiCol_Separator] = ImVec4(0.30f, 0.30f, 0.35f, 0.60f);
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.70f);
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);

    // Resize grip
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.26f, 0.59f, 0.98f, 0.25f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);

    // Tab
    colors[ImGuiCol_Tab] = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.26f, 0.50f, 0.74f, 0.80f);
    colors[ImGuiCol_TabActive] = ImVec4(0.20f, 0.40f, 0.60f, 1.00f);
    colors[ImGuiCol_TabUnfocused] = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.16f, 0.30f, 0.44f, 1.00f);

    // Docking colors
    colors[ImGuiCol_DockingPreview] = ImVec4(0.26f, 0.59f, 0.98f, 0.70f);
    colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.08f, 0.08f, 0.10f, 1.00f);

    // Plot lines
    colors[ImGuiCol_PlotLines] = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
    colors[ImGuiCol_PlotHistogram] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.36f, 0.69f, 1.00f, 1.00f);

    // Table
    colors[ImGuiCol_TableHeaderBg] = ImVec4(0.16f, 0.16f, 0.18f, 1.00f);
    colors[ImGuiCol_TableBorderStrong] = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.22f, 0.22f, 0.26f, 1.00f);
    colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(1.00f, 1.00f, 1.00f, 0.03f);

    // Text
    colors[ImGuiCol_Text] = ImVec4(0.92f, 0.92f, 0.94f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.52f, 1.00f);
    colors[ImGuiCol_TextSelectedBg] = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);

    // Drag/drop
    colors[ImGuiCol_DragDropTarget] = ImVec4(0.26f, 0.59f, 0.98f, 0.90f);

    // Navigation
    colors[ImGuiCol_NavHighlight] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
    colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
    colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);

    // Modal window
    colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.60f);
}

void DX12Renderer::InitializeImGui(HWND hwnd)
{
    std::cout << "  Initializing ImGui..." << std::endl;

    // Create descriptor heap for ImGui
    D3D12_DESCRIPTOR_HEAP_DESC desc = {};
    desc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    desc.NumDescriptors = 1;
    desc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(device->CreateDescriptorHeap(&desc, IID_PPV_ARGS(&imguiSrvHeap)));

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Dear ImGui style with modern look
    ApplyModernStyle();

    // Build font atlas BEFORE initializing DX12 backend
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX12_Init(device.Get(), 2,
        DXGI_FORMAT_R8G8B8A8_UNORM, imguiSrvHeap.Get(),
        imguiSrvHeap->GetCPUDescriptorHandleForHeapStart(),
        imguiSrvHeap->GetGPUDescriptorHandleForHeapStart());

    std::cout << "  ImGui initialized successfully." << std::endl;
}

void DX12Renderer::ShutdownImGui()
{
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void DX12Renderer::RenderImGui(PxScene* scene, float deltaTime, PxCudaContextManager* cudaContextManager,
                               SimulationRecorder* recorder, PerformanceProfiler* profiler,
                               ExperimentController* controller, PxMaterial* material)
{
    // Start the Dear ImGui frame
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    // Create a fullscreen dock space (Unity-style layout)
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
        ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar(3);

    ImGuiID dockspace_id = ImGui::GetID("MainDockSpace");

    // Set up initial dock layout (Unity-style: panels on sides, simulation in center)
    if (!m_dockLayoutInitialized)
    {
        m_dockLayoutInitialized = true;

        // Clear any existing layout
        ImGui::DockBuilderRemoveNode(dockspace_id);
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

        // Split the dock space:
        // [Left Panel (20%)] [Center - Scene View (60%)] [Right Panel (20%)]
        //                    [Bottom Panel (25%)]
        ImGuiID dock_left, dock_right, dock_bottom, dock_center;

        // First split: left panel (20%) | rest (80%)
        ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.20f, &dock_left, &dock_center);

        // Split rest: center+bottom (75%) | right panel (25% of remaining = 20% total)
        ImGui::DockBuilderSplitNode(dock_center, ImGuiDir_Right, 0.25f, &dock_right, &dock_center);

        // Split center: scene view (75%) | bottom panel (25%)
        ImGui::DockBuilderSplitNode(dock_center, ImGuiDir_Down, 0.25f, &dock_bottom, &dock_center);

        // Dock windows to their positions
        // Left panel: Scene selection and control
        ImGui::DockBuilderDockWindow("Physics Debug", dock_left);
        ImGui::DockBuilderDockWindow("Scene Control", dock_left);

        // Right panel: Visualization options
        ImGui::DockBuilderDockWindow("Visualization", dock_right);
        ImGui::DockBuilderDockWindow("Thresholds", dock_right);

        // Bottom panel: Recording and performance
        ImGui::DockBuilderDockWindow("Recording", dock_bottom);
        ImGui::DockBuilderDockWindow("Performance", dock_bottom);

        ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::End();

    // ==================== LEFT PANEL: Physics Debug ====================
    ImGui::Begin("Physics Debug");

    // FPS and frame time
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Frame Time: %.3f ms", deltaTime * 1000.0f);
    ImGui::Separator();

    // GPU PhysX Status
    ImGui::Text("GPU PhysX (CUDA):");
    if (cudaContextManager && cudaContextManager->contextIsValid())
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "ENABLED");
    }
    else
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "DISABLED");
    }

    // PhysX statistics
    if (scene)
    {
        ImGui::Separator();
        PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
        PxU32 nbDynamic = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
        PxU32 nbStatic = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);

        ImGui::Text("Actors: %d (Dyn: %d, Sta: %d)", nbActors, nbDynamic, nbStatic);
    }

    // Camera info
    ImGui::Separator();
    XMFLOAT3 camPos = camera.GetPosition();
    ImGui::Text("Camera: (%.1f, %.1f, %.1f)", camPos.x, camPos.y, camPos.z);

    // Mesh cache
    ImGui::Text("Mesh Cache: %zu", meshCache.size());

    ImGui::End();

    // ==================== LEFT PANEL: Scene Control ====================
    ImGui::Begin("Scene Control");

    // Scene Selection
    const char* sceneNames[] = {
        "Basic Stack",
        "Shapes Variety",
        "Falling Objects",
        "Pyramid",
        "Stress Test (400)",
        "Snippet HelloWorld",
        "Pendulum (Revolute Joint)",
        "Chain (Spherical Joints)",
        "Mechanism (Mixed Joints)",
        "Breakable Wall",
        "Robot Arm (Articulation)",
        "Convex Shapes",
        "Terrain (Heightfield)",
        "Soft Body (GPU)"
    };

    int currentScene = static_cast<int>(gCurrentSceneType);
    if (ImGui::Combo("Scene", &currentScene, sceneNames, IM_ARRAYSIZE(sceneNames)))
    {
        switchScene(static_cast<SceneType>(currentScene));
    }

    ImGui::Separator();

    // Simulation Controls
    if (controller)
    {
        bool isPaused = controller->IsPaused();
        if (ImGui::Checkbox("Paused", &isPaused))
        {
            controller->TogglePause();
        }

        float timeScale = controller->GetTimeScale();
        if (ImGui::SliderFloat("Time Scale", &timeScale, 0.1f, 5.0f))
        {
            controller->SetTimeScale(timeScale);
        }

        if (ImGui::Button("Reset Scene"))
        {
            controller->RequestReset();
        }
    }

    ImGui::Separator();

    // Gravity control
    if (scene)
    {
        PxVec3 gravity = scene->getGravity();
        float gravityY = gravity.y;
        if (ImGui::SliderFloat("Gravity Y", &gravityY, -20.0f, 0.0f))
        {
            scene->setGravity(PxVec3(gravity.x, gravityY, gravity.z));
        }
    }

    // Continuous spawning
    ImGui::Separator();
    ImGui::Checkbox("Continuous Spawn", &gEnableContinuousSpawn);
    if (gEnableContinuousSpawn)
    {
        ImGui::SliderFloat("Spawn Interval", &gSpawnInterval, 0.1f, 5.0f);
    }

    ImGui::End();

    // ==================== RIGHT PANEL: Visualization ====================
    ImGui::Begin("Visualization");

    // Instanced rendering
    ImGui::Checkbox("Instanced Rendering", &m_useInstancing);

    ImGui::Separator();

    // Velocity vectors
    ImGui::Checkbox("Velocity Vectors", &m_showVelocityVectors);
    if (m_showVelocityVectors)
    {
        ImGui::SliderFloat("Velocity Scale", &m_velocityScale, 0.1f, 5.0f);
    }

    // Collision points
    ImGui::Checkbox("Collision Points", &m_showCollisions);
    if (m_showCollisions)
    {
        ImGui::Text("  Active: %zu", m_collisionPoints.size());
        if (ImGui::Button("Clear##Collisions"))
        {
            ClearCollisionPoints();
        }
    }

    // Trajectory trails
    ImGui::Checkbox("Trajectory Trails", &m_showTrajectories);
    if (m_showTrajectories)
    {
        ImGui::Text("  Active: %zu", m_trajectories.size());
        ImGui::SliderFloat("Duration", &m_trajectoryMaxDuration, 0.5f, 10.0f);

        const char* filterModes[] = { "All", "Selected", "Fastest N", "Recent N" };
        int currentFilterMode = static_cast<int>(m_trajectoryFilterMode);
        if (ImGui::Combo("Filter", &currentFilterMode, filterModes, 4))
        {
            m_trajectoryFilterMode = static_cast<TrajectoryFilterMode>(currentFilterMode);
        }

        if (m_trajectoryFilterMode == TrajectoryFilterMode::FASTEST_N ||
            m_trajectoryFilterMode == TrajectoryFilterMode::MOST_RECENT_N)
        {
            ImGui::SliderInt("Max Count", &m_trajectoryMaxCount, 1, 50);
        }

        if (ImGui::Button("Clear##Trajectories"))
        {
            ClearTrajectories();
        }
    }

    // Force visualization
    ImGui::Checkbox("Forces", &m_showForces);
    if (m_showForces)
    {
        ImGui::SliderFloat("Force Scale", &m_forceScale, 0.001f, 0.1f);
        ImGui::Text("  Active: %zu", m_contactForces.size());

        ImGui::Checkbox("Contact Normals", &m_showContactNormals);
        if (m_showContactNormals)
        {
            ImGui::SliderFloat("Normal Scale", &m_contactNormalScale, 0.1f, 5.0f);
        }
    }

    ImGui::Separator();

    // Heatmap
    const char* heatmapModes[] = { "None", "Velocity", "Kinetic Energy" };
    int currentMode = static_cast<int>(m_heatmapMode);
    if (ImGui::Combo("Heatmap", &currentMode, heatmapModes, 3))
    {
        m_heatmapMode = static_cast<HeatmapMode>(currentMode);
    }

    if (m_heatmapMode != HeatmapMode::NONE)
    {
        ImGui::Checkbox("Auto-Scale", &m_heatmapAutoScale);
        if (!m_heatmapAutoScale)
        {
            ImGui::SliderFloat("Min", &m_heatmapMin, 0.0f, 50.0f);
            ImGui::SliderFloat("Max", &m_heatmapMax, 0.0f, 100.0f);
        }
        else
        {
            ImGui::Text("Range: %.1f - %.1f", m_heatmapMin, m_heatmapMax);
        }
    }

    ImGui::End();

    // ==================== RIGHT PANEL: Thresholds ====================
    ImGui::Begin("Thresholds");

    ImGui::Text("Velocity (m/s):");
    ImGui::PushItemWidth(80);
    ImGui::DragFloat("Slow##vel", &m_thresholds.velocitySlow, 0.5f, 0.1f, 50.0f, "%.1f");
    ImGui::SameLine();
    ImGui::DragFloat("Fast##vel", &m_thresholds.velocityFast, 0.5f, 0.1f, 100.0f, "%.1f");
    ImGui::PopItemWidth();
    if (m_thresholds.velocitySlow > m_thresholds.velocityFast)
        m_thresholds.velocitySlow = m_thresholds.velocityFast;

    ImGui::Separator();

    ImGui::Text("Force (N):");
    ImGui::PushItemWidth(80);
    ImGui::DragFloat("Small##force", &m_thresholds.forceSmall, 10.0f, 1.0f, 1000.0f, "%.0f");
    ImGui::SameLine();
    ImGui::DragFloat("Large##force", &m_thresholds.forceLarge, 10.0f, 1.0f, 5000.0f, "%.0f");
    ImGui::PopItemWidth();
    if (m_thresholds.forceSmall > m_thresholds.forceLarge)
        m_thresholds.forceSmall = m_thresholds.forceLarge;

    ImGui::Separator();

    ImGui::Text("Impulse:");
    ImGui::PushItemWidth(80);
    ImGui::DragFloat("Low##impulse", &m_thresholds.impulseLow, 1.0f, 0.1f, 100.0f, "%.1f");
    ImGui::SameLine();
    ImGui::DragFloat("High##impulse", &m_thresholds.impulseHigh, 1.0f, 0.1f, 500.0f, "%.1f");
    ImGui::PopItemWidth();
    if (m_thresholds.impulseLow > m_thresholds.impulseHigh)
        m_thresholds.impulseLow = m_thresholds.impulseHigh;

    ImGui::Separator();

    if (ImGui::Button("Reset Defaults"))
    {
        m_thresholds = VisualizationThresholds();
    }

    // Performance Settings
    ImGui::Separator();
    ImGui::Text("Performance:");
    ImGui::Checkbox("Adaptive Viz", &m_adaptiveVisualization);
    if (m_adaptiveVisualization)
    {
        ImGui::SliderFloat("Target FPS", &m_targetVisualizationFPS, 15.0f, 60.0f, "%.0f");
    }

    ImGui::End();

    // ==================== BOTTOM PANEL: Recording ====================
    ImGui::Begin("Recording");

    if (recorder)
    {
        bool isRecording = recorder->IsRecording();
        bool recPaused = recorder->IsPaused();

        // Status indicator
        if (isRecording && !recPaused)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "REC");
        }
        else if (recPaused)
        {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "PAUSED");
        }
        else
        {
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Stopped");
        }

        ImGui::SameLine();

        // Controls
        if (isRecording)
        {
            if (ImGui::Button("Stop"))
            {
                recorder->StopRecording();
            }
            ImGui::SameLine();
            if (recPaused)
            {
                if (ImGui::Button("Resume"))
                {
                    recorder->ResumeRecording();
                }
            }
            else
            {
                if (ImGui::Button("Pause"))
                {
                    recorder->PauseRecording();
                }
            }
        }
        else
        {
            if (ImGui::Button("Start Recording"))
            {
                recorder->StartRecording();
            }
        }

        // Stats
        ImGui::SameLine();
        ImGui::Text("| Frames: %zu | %.1fs | Collisions: %zu",
            recorder->GetFrameCount(),
            recorder->GetRecordingDuration(),
            recorder->GetTotalCollisions());

        // Export
        ImGui::SameLine();
        if (ImGui::Button("CSV"))
        {
            time_t now = time(nullptr);
            struct tm timeinfo;
            localtime_s(&timeinfo, &now);
            char timestamp[32];
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
            std::string filename = std::string("simulation_") + timestamp + ".csv";
            recorder->ExportToCSV(filename);
        }
        ImGui::SameLine();
        if (ImGui::Button("JSON"))
        {
            time_t now = time(nullptr);
            struct tm timeinfo;
            localtime_s(&timeinfo, &now);
            char timestamp[32];
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
            std::string filename = std::string("simulation_") + timestamp + ".json";
            recorder->ExportToJSON(filename);
        }
    }

    ImGui::End();

    // ==================== BOTTOM PANEL: Performance ====================
    // Update performance history
    if (profiler)
    {
        m_perfHistory.AddSample(
            profiler->GetCurrentFrameTime(),
            profiler->GetCurrentPhysicsTime(),
            profiler->GetCurrentRenderTime(),
            profiler->GetCurrentFPS()
        );
    }

    ImGui::Begin("Performance");

    if (profiler)
    {
        // Current stats in a compact row
        ImGui::Text("Frame: %.2fms | Physics: %.2fms | Render: %.2fms | FPS: %.0f",
            profiler->GetCurrentFrameTime(),
            profiler->GetCurrentPhysicsTime(),
            profiler->GetCurrentRenderTime(),
            profiler->GetCurrentFPS());

        // Graphs (compact)
        if (!m_perfHistory.fpsHistory.empty())
        {
            float fpsMax = *(std::max_element)(m_perfHistory.fpsHistory.begin(), m_perfHistory.fpsHistory.end()) * 1.2f;
            ImGui::PlotLines("##FPS", m_perfHistory.fpsHistory.data(),
                static_cast<int>(m_perfHistory.fpsHistory.size()),
                0, "FPS", 0.0f, fpsMax, ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 5, 50));

            ImGui::SameLine();

            float frameTimeMax = *(std::max_element)(m_perfHistory.frameTimeHistory.begin(),
                m_perfHistory.frameTimeHistory.end()) * 1.2f;
            ImGui::PlotLines("##Frame", m_perfHistory.frameTimeHistory.data(),
                static_cast<int>(m_perfHistory.frameTimeHistory.size()),
                0, "Frame(ms)", 0.0f, frameTimeMax, ImVec2(ImGui::GetContentRegionAvail().x, 50));
        }
    }

    ImGui::End();

    // Rendering
    ImGui::Render();

    // Set ImGui descriptor heap
    ID3D12DescriptorHeap* heaps[] = { imguiSrvHeap.Get() };
    commandList->SetDescriptorHeaps(1, heaps);

    // Render ImGui
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), commandList.Get());
}

//=============================================================================
// Instanced Rendering Implementation
//=============================================================================

void DX12Renderer::CreateInstanceBuffer(UINT maxInstances)
{
    // Create instance buffer for storing instance data
    const UINT instanceBufferSize = (sizeof(InstanceData) * maxInstances + 255) & ~255;

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(instanceBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&instanceBuffer)
    ));

    // Map instance buffer
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(instanceBuffer->Map(0, &readRange, reinterpret_cast<void**>(&instanceBufferData)));

    instanceBufferView.BufferLocation = instanceBuffer->GetGPUVirtualAddress();
    instanceBufferView.SizeInBytes = instanceBufferSize;
    instanceBufferView.StrideInBytes = sizeof(InstanceData);
}

void DX12Renderer::RenderPhysXSceneInstanced(PxScene* scene)
{
    if (!scene) return;

    // Get all actors
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    // Group actors by mesh type for batched rendering
    std::unordered_map<std::string, InstanceBatch> batches;

    for (PxU32 i = 0; i < nbActors; i++)
    {
        PxRigidActor* actor = actors[i];
        if (!actor) continue;

        // Determine color based on actor type
        XMFLOAT3 color(0.5f, 0.5f, 0.5f);
        if (actor->is<PxRigidDynamic>())
        {
            color = XMFLOAT3(0.3f, 0.7f, 0.3f); // Green for dynamic
        }
        else if (actor->is<PxRigidStatic>())
        {
            color = XMFLOAT3(0.6f, 0.6f, 0.6f); // Gray for static
        }

        PxU32 nShapes = actor->getNbShapes();
        if (nShapes == 0) continue;

        std::vector<PxShape*> shapes(nShapes);
        actor->getShapes(shapes.data(), nShapes);

        for (PxU32 j = 0; j < nShapes; j++)
        {
            PxShape* shape = shapes[j];
            if (!shape) continue;

            // Get transform
            PxTransform pxTransform = PxShapeExt::getGlobalPose(*shape, *actor);
            PxMat44 pxMat(pxTransform);

            // Convert PhysX matrix to DirectXMath
            XMMATRIX world = XMMatrixSet(
                pxMat[0][0], pxMat[0][1], pxMat[0][2], pxMat[0][3],
                pxMat[1][0], pxMat[1][1], pxMat[1][2], pxMat[1][3],
                pxMat[2][0], pxMat[2][1], pxMat[2][2], pxMat[2][3],
                pxMat[3][0], pxMat[3][1], pxMat[3][2], pxMat[3][3]
            );

            // Get geometry
            PxGeometryHolder geomHolder = shape->getGeometry();
            PxGeometryType::Enum geomType = geomHolder.getType();

            std::string batchKey;
            std::shared_ptr<RenderMesh> meshPtr;

            switch (geomType)
            {
            case PxGeometryType::eBOX:
            {
                const PxBoxGeometry& boxGeom = geomHolder.box();
                batchKey = GenerateBoxCacheKey(boxGeom);
                meshPtr = GetOrCreateBoxMesh(boxGeom, color);
                break;
            }
            case PxGeometryType::eSPHERE:
            {
                const PxSphereGeometry& sphereGeom = geomHolder.sphere();
                batchKey = GenerateSphereCacheKey(sphereGeom);
                meshPtr = GetOrCreateSphereMesh(sphereGeom, color);
                break;
            }
            case PxGeometryType::ePLANE:
            {
                // Skip rendering infinite planes - we have a visible ground box instead
                // Infinite planes don't have proper transforms and cause rendering issues
                continue;
            }
            default:
                continue;
            }

            // Add to batch
            if (meshPtr)
            {
                if (batches.find(batchKey) == batches.end())
                {
                    batches[batchKey].mesh = meshPtr;
                }

                InstanceData instanceData;
                instanceData.world = world;
                instanceData.color = XMFLOAT4(color.x, color.y, color.z, 1.0f);
                batches[batchKey].instances.push_back(instanceData);
            }
        }
    }

    // Render all batches with object index tracking
    UINT objectIndex = 0;
    for (auto& pair : batches)
    {
        RenderInstanceBatch(pair.second, objectIndex);
    }

    // Render velocity vectors if enabled
    if (m_showVelocityVectors)
    {
        RenderVelocityVectors(scene, objectIndex);
    }

    // Render collision points if enabled
    if (m_showCollisions)
    {
        RenderCollisionPoints(objectIndex);
    }

    // Render trajectories if enabled
    if (m_showTrajectories)
    {
        RenderTrajectories(objectIndex);
    }

    // Render deformable volumes (soft bodies)
    if (gDeformableManager)
    {
        RenderDeformableVolumes(gDeformableManager.get(), objectIndex);
    }
}

void DX12Renderer::RenderInstanceBatch(const InstanceBatch& batch, UINT& objectIndex)
{
    if (!batch.mesh || batch.instances.empty()) return;

    // Render each instance individually with proper constant buffer offset
    for (const auto& instance : batch.instances)
    {
        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            static bool warningShown = false;
            if (!warningShown)
            {
                std::cerr << "WARNING: Too many objects in instanced rendering (" << objectIndex
                          << ") - exceeding constant buffer capacity. Skipping remaining objects." << std::endl;
                warningShown = true;
            }
            return;  // Skip rendering remaining objects
        }

        UpdateConstantBuffer(instance.world, objectIndex);

        // Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        commandList->IASetVertexBuffers(0, 1, &batch.mesh->vertexBufferView);
        commandList->IASetIndexBuffer(&batch.mesh->indexBufferView);
        commandList->DrawIndexedInstanced(batch.mesh->indexCount, 1, 0, 0, 0);

        objectIndex++;
    }
}


//=============================================================================
// Force Visualization Implementation
//=============================================================================

void DX12Renderer::AddContactForce(const PxVec3& position, const PxVec3& force)
{
    // Call the version with normal, defaulting to zero normal
    AddContactForceWithNormal(position, force, PxVec3(0, 0, 0));
}

void DX12Renderer::AddContactForceWithNormal(const PxVec3& position, const PxVec3& force, const PxVec3& normal)
{
    std::lock_guard<std::mutex> lock(m_contactForceMutex);
    ContactForce cf;
    cf.position = XMFLOAT3(position.x, position.y, position.z);
    cf.force = XMFLOAT3(force.x, force.y, force.z);
    cf.normal = XMFLOAT3(normal.x, normal.y, normal.z);
    cf.timeStamp = static_cast<float>(ImGui::GetTime());
    m_contactForces.push_back(cf);
}

void DX12Renderer::RenderForces(PxScene* scene, UINT& objectIndex)
{
    if (!m_showForces) return;

    // Render contact forces
    RenderContactForces(objectIndex);

    // Render contact normals (if enabled)
    RenderContactNormals(objectIndex);

    // Render gravity forces for all dynamic actors
    RenderGravityForces(scene, objectIndex);
}

void DX12Renderer::RenderContactForces(UINT& objectIndex)
{
    // PERFORMANCE OPTIMIZATION: Only update visualization every N frames
    static int forceFrameCounter = 0;
    const int VISUALIZATION_UPDATE_INTERVAL = RenderConstants::kVisualizationUpdateInterval;
    forceFrameCounter++;
    if (forceFrameCounter % VISUALIZATION_UPDATE_INTERVAL != 0)
    {
        return;  // Skip this frame
    }

    std::lock_guard<std::mutex> lock(m_contactForceMutex);

    float currentTime = static_cast<float>(ImGui::GetTime());

    // Remove old contact forces
    m_contactForces.erase(
        std::remove_if(m_contactForces.begin(), m_contactForces.end(),
            [this, currentTime](const ContactForce& cf) {
                return (currentTime - cf.timeStamp) > m_forceDisplayDuration;
            }),
        m_contactForces.end()
    );

    // Render contact force arrows
    // PERFORMANCE OPTIMIZATION: Limit the number of force arrows
    const int MAX_FORCE_ARROWS = RenderConstants::kMaxForceArrows;
    int forceCount = 0;

    for (const auto& cf : m_contactForces)
    {
        if (forceCount >= MAX_FORCE_ARROWS) break;
        XMFLOAT3 start = cf.position;
        XMFLOAT3 end = XMFLOAT3(
            start.x + cf.force.x * m_forceScale,
            start.y + cf.force.y * m_forceScale,
            start.z + cf.force.z * m_forceScale
        );

        // Color based on force magnitude using configurable thresholds
        float forceMag = sqrtf(cf.force.x * cf.force.x + cf.force.y * cf.force.y + cf.force.z * cf.force.z);
        XMFLOAT3 color;
        if (forceMag < m_thresholds.forceSmall)
        {
            color = XMFLOAT3(0.0f, 1.0f, 0.0f); // Green for small forces
        }
        else if (forceMag < m_thresholds.forceLarge)
        {
            color = XMFLOAT3(1.0f, 1.0f, 0.0f); // Yellow for medium forces
        }
        else
        {
            color = XMFLOAT3(1.0f, 0.0f, 0.0f); // Red for large forces
        }

        // Create and render the force arrow with arrow head
        auto arrowMesh = std::make_unique<RenderMesh>(CreateArrowMesh(start, end, color));

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;  // Skip rendering remaining forces
        }

        XMMATRIX world = XMMatrixIdentity();
        UpdateConstantBuffer(world, objectIndex);

        // Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        // Draw the line part first (indices 0-1)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        commandList->IASetVertexBuffers(0, 1, &arrowMesh->vertexBufferView);
        commandList->IASetIndexBuffer(&arrowMesh->indexBufferView);
        commandList->DrawIndexedInstanced(2, 1, 0, 0, 0);

        // Draw the cone part (remaining indices)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        commandList->DrawIndexedInstanced(arrowMesh->indexCount - 2, 1, 2, 0, 0);

        dynamicMeshCache[frameIndex].push_back(std::move(arrowMesh));
        objectIndex++;
        forceCount++;
    }
}

void DX12Renderer::RenderContactNormals(UINT& objectIndex)
{
    if (!m_showContactNormals) return;

    // PERFORMANCE OPTIMIZATION: Only update visualization every N frames
    static int normalFrameCounter = 0;
    const int VISUALIZATION_UPDATE_INTERVAL = RenderConstants::kVisualizationUpdateInterval;
    normalFrameCounter++;
    if (normalFrameCounter % VISUALIZATION_UPDATE_INTERVAL != 0)
    {
        return;  // Skip this frame
    }

    std::lock_guard<std::mutex> lock(m_contactForceMutex);

    float currentTime = static_cast<float>(ImGui::GetTime());

    // Cyan color for contact normals
    const XMFLOAT3 normalColor(0.0f, 1.0f, 1.0f);

    // Limit the number of normal arrows for performance
    const int MAX_NORMAL_ARROWS = RenderConstants::kMaxForceArrows;
    int normalCount = 0;

    for (const auto& cf : m_contactForces)
    {
        if (normalCount >= MAX_NORMAL_ARROWS) break;

        // Skip expired contact forces
        if ((currentTime - cf.timeStamp) > m_forceDisplayDuration) continue;

        // Skip if normal is zero (not provided)
        float normalMag = sqrtf(cf.normal.x * cf.normal.x + cf.normal.y * cf.normal.y + cf.normal.z * cf.normal.z);
        if (normalMag < 0.001f) continue;

        // Normalize and scale the normal vector
        XMFLOAT3 normalDir(
            cf.normal.x / normalMag,
            cf.normal.y / normalMag,
            cf.normal.z / normalMag
        );

        XMFLOAT3 start = cf.position;
        XMFLOAT3 end = XMFLOAT3(
            start.x + normalDir.x * m_contactNormalScale,
            start.y + normalDir.y * m_contactNormalScale,
            start.z + normalDir.z * m_contactNormalScale
        );

        // Create and render the normal arrow with cyan color
        auto arrowMesh = std::make_unique<RenderMesh>(CreateArrowMesh(start, end, normalColor, 0.08f, 0.15f));

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;  // Skip rendering remaining normals
        }

        XMMATRIX world = XMMatrixIdentity();
        UpdateConstantBuffer(world, objectIndex);

        // Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        // Draw the line part first (indices 0-1)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        commandList->IASetVertexBuffers(0, 1, &arrowMesh->vertexBufferView);
        commandList->IASetIndexBuffer(&arrowMesh->indexBufferView);
        commandList->DrawIndexedInstanced(2, 1, 0, 0, 0);

        // Draw the cone part (remaining indices)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        commandList->DrawIndexedInstanced(arrowMesh->indexCount - 2, 1, 2, 0, 0);

        dynamicMeshCache[frameIndex].push_back(std::move(arrowMesh));
        objectIndex++;
        normalCount++;
    }
}

void DX12Renderer::RenderGravityForces(PxScene* scene, UINT& objectIndex)
{
    if (!scene) return;

    // PERFORMANCE OPTIMIZATION: Only update visualization every N frames
    static int gravityFrameCounter = 0;
    const int VISUALIZATION_UPDATE_INTERVAL = RenderConstants::kVisualizationUpdateInterval;
    gravityFrameCounter++;
    if (gravityFrameCounter % VISUALIZATION_UPDATE_INTERVAL != 0)
    {
        return;  // Skip this frame
    }

    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    PxVec3 gravity = scene->getGravity();

    // PERFORMANCE OPTIMIZATION: Limit the number of gravity arrows
    const int MAX_GRAVITY_ARROWS = RenderConstants::kMaxGravityArrows;
    int gravityCount = 0;

    for (PxU32 i = 0; i < nbActors; i++)
    {
        if (gravityCount >= MAX_GRAVITY_ARROWS) break;
        PxRigidDynamic* dynamicActor = actors[i]->is<PxRigidDynamic>();
        if (!dynamicActor) continue;

        // Get actor mass
        float mass = dynamicActor->getMass();
        if (mass < 0.001f) continue;  // Skip very light objects

        // Calculate gravity force
        PxVec3 gravityForce = gravity * mass;

        // Get actor position
        PxTransform transform = dynamicActor->getGlobalPose();
        XMFLOAT3 start(transform.p.x, transform.p.y, transform.p.z);
        XMFLOAT3 end(
            start.x + gravityForce.x * m_forceScale,
            start.y + gravityForce.y * m_forceScale,
            start.z + gravityForce.z * m_forceScale
        );

        // Gravity forces are shown in blue
        XMFLOAT3 color(0.0f, 0.5f, 1.0f);

        // Create and render the gravity arrow with arrow head
        auto arrowMesh = std::make_unique<RenderMesh>(CreateArrowMesh(start, end, color));

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;  // Skip rendering remaining gravity forces
        }

        XMMATRIX world = XMMatrixIdentity();
        UpdateConstantBuffer(world, objectIndex);

        // Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        // Draw the line part first (indices 0-1)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        commandList->IASetVertexBuffers(0, 1, &arrowMesh->vertexBufferView);
        commandList->IASetIndexBuffer(&arrowMesh->indexBufferView);
        commandList->DrawIndexedInstanced(2, 1, 0, 0, 0);

        // Draw the cone part (remaining indices)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        commandList->DrawIndexedInstanced(arrowMesh->indexCount - 2, 1, 2, 0, 0);

        dynamicMeshCache[frameIndex].push_back(std::move(arrowMesh));
        objectIndex++;
        gravityCount++;
    }
}


//=============================================================================
// Additional Visualization Methods
//=============================================================================

void DX12Renderer::RenderVelocityVectors(PxScene* scene, UINT& objectIndex)
{
    if (!scene) return;

    // PERFORMANCE OPTIMIZATION: Only update visualization every N frames to reduce GPU resource creation
    static int visualizationFrameCounter = 0;
    const int VISUALIZATION_UPDATE_INTERVAL = RenderConstants::kVisualizationUpdateInterval;  // Update every 10 frames
    visualizationFrameCounter++;
    if (visualizationFrameCounter % VISUALIZATION_UPDATE_INTERVAL != 0)
    {
        return;  // Skip this frame
    }

    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    // PERFORMANCE OPTIMIZATION: Limit the number of vectors to render
    const int MAX_VELOCITY_VECTORS = RenderConstants::kMaxVelocityVectors;
    int vectorCount = 0;

    for (PxU32 i = 0; i < nbActors; i++)
    {
        // PERFORMANCE OPTIMIZATION: Limit vector count
        if (vectorCount >= MAX_VELOCITY_VECTORS) break;

        PxRigidDynamic* dynamicActor = actors[i]->is<PxRigidDynamic>();
        if (!dynamicActor) continue;

        PxVec3 velocity = dynamicActor->getLinearVelocity();
        float speed = velocity.magnitude();

        if (speed < 0.1f) continue;  // Skip slow-moving objects

        vectorCount++;

        PxTransform transform = dynamicActor->getGlobalPose();
        XMFLOAT3 start(transform.p.x, transform.p.y, transform.p.z);
        XMFLOAT3 end(
            start.x + velocity.x * m_velocityScale,
            start.y + velocity.y * m_velocityScale,
            start.z + velocity.z * m_velocityScale
        );

        // Color based on speed using configurable thresholds
        XMFLOAT3 color;
        if (speed < m_thresholds.velocitySlow)
        {
            color = XMFLOAT3(0.0f, 0.5f, 1.0f); // Blue for slow
        }
        else if (speed < m_thresholds.velocityFast)
        {
            color = XMFLOAT3(1.0f, 1.0f, 0.0f); // Yellow for medium
        }
        else
        {
            color = XMFLOAT3(1.0f, 0.0f, 0.0f); // Red for fast
        }

        // Create and render the velocity arrow with arrow head
        auto arrowMesh = std::make_unique<RenderMesh>(CreateArrowMesh(start, end, color));

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;  // Skip rendering remaining vectors
        }

        XMMATRIX world = XMMatrixIdentity();
        UpdateConstantBuffer(world, objectIndex);

        // Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        // Draw the line part first (indices 0-1)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        commandList->IASetVertexBuffers(0, 1, &arrowMesh->vertexBufferView);
        commandList->IASetIndexBuffer(&arrowMesh->indexBufferView);
        commandList->DrawIndexedInstanced(2, 1, 0, 0, 0);  // First 2 indices are the line

        // Draw the cone part (remaining indices)
        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        commandList->DrawIndexedInstanced(arrowMesh->indexCount - 2, 1, 2, 0, 0);  // Skip first 2 indices

        dynamicMeshCache[frameIndex].push_back(std::move(arrowMesh));
        objectIndex++;
    }
}

void DX12Renderer::AddCollisionPoint(const PxVec3& point, float impulse)
{
    std::lock_guard<std::mutex> lock(m_collisionMutex);
    CollisionPoint cp;
    cp.position = XMFLOAT3(point.x, point.y, point.z);
    cp.impulse = impulse;
    cp.timeStamp = static_cast<float>(ImGui::GetTime());
    m_collisionPoints.push_back(cp);
}

void DX12Renderer::RenderCollisionPoints(UINT& objectIndex)
{
    // PERFORMANCE OPTIMIZATION: Only update visualization every N frames
    static int collisionFrameCounter = 0;
    const int VISUALIZATION_UPDATE_INTERVAL = RenderConstants::kVisualizationUpdateInterval;
    collisionFrameCounter++;
    if (collisionFrameCounter % VISUALIZATION_UPDATE_INTERVAL != 0)
    {
        return;  // Skip this frame
    }

    std::lock_guard<std::mutex> lock(m_collisionMutex);

    float currentTime = static_cast<float>(ImGui::GetTime());

    // Remove old collision points
    m_collisionPoints.erase(
        std::remove_if(m_collisionPoints.begin(), m_collisionPoints.end(),
            [this, currentTime](const CollisionPoint& cp) {
                return (currentTime - cp.timeStamp) > m_collisionDisplayDuration;
            }),
        m_collisionPoints.end()
    );

    // Render collision point spheres
    // PERFORMANCE OPTIMIZATION: Limit the number of collision points
    const int MAX_COLLISION_POINTS = RenderConstants::kMaxCollisionPoints;
    int pointCount = 0;

    for (const auto& cp : m_collisionPoints)
    {
        if (pointCount >= MAX_COLLISION_POINTS) break;
        float age = currentTime - cp.timeStamp;
        float alpha = 1.0f - (age / m_collisionDisplayDuration);

        // Color based on impulse magnitude using configurable thresholds
        XMFLOAT3 color;
        if (cp.impulse < m_thresholds.impulseLow)
        {
            color = XMFLOAT3(0.0f, 1.0f * alpha, 0.0f);  // Green for low impulse
        }
        else if (cp.impulse < m_thresholds.impulseHigh)
        {
            color = XMFLOAT3(1.0f * alpha, 1.0f * alpha, 0.0f);  // Yellow for medium impulse
        }
        else
        {
            color = XMFLOAT3(1.0f * alpha, 0.0f, 0.0f);  // Red for high impulse
        }

        // Create sphere with size scaled by impulse (0.05 to 0.30 range)
        float normalizedImpulse = std::clamp(cp.impulse / m_thresholds.impulseHigh, 0.0f, 1.0f);
        float radius = 0.05f + normalizedImpulse * 0.25f;
        PxSphereGeometry sphereGeom(radius);
        auto sphereMesh = GetOrCreateSphereMesh(sphereGeom, color);

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;  // Skip rendering remaining collision points
        }

        XMMATRIX world = XMMatrixTranslation(cp.position.x, cp.position.y, cp.position.z);
        UpdateConstantBuffer(world, objectIndex);

        // Bind constant buffer with offset for this specific object
        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        commandList->IASetVertexBuffers(0, 1, &sphereMesh->vertexBufferView);
        commandList->IASetIndexBuffer(&sphereMesh->indexBufferView);
        commandList->DrawIndexedInstanced(sphereMesh->indexCount, 1, 0, 0, 0);

        objectIndex++;
        pointCount++;
    }
}

void DX12Renderer::UpdateTrajectories(PxScene* scene, float deltaTime)
{
    if (!scene) return;

    float currentTime = static_cast<float>(ImGui::GetTime());

    // Check if it's time to update trajectories
    if (currentTime - m_lastTrajectoryUpdate < m_trajectoryUpdateInterval)
        return;

    m_lastTrajectoryUpdate = currentTime;

    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    for (PxU32 i = 0; i < nbActors; i++)
    {
        PxRigidDynamic* dynamicActor = actors[i]->is<PxRigidDynamic>();
        if (!dynamicActor) continue;

        uint32_t actorID = reinterpret_cast<uintptr_t>(dynamicActor) & 0xFFFFFFFF;
        PxTransform transform = dynamicActor->getGlobalPose();

        TrajectoryPoint point;
        point.position = XMFLOAT3(transform.p.x, transform.p.y, transform.p.z);
        point.timeStamp = currentTime;

        // Create trajectory if it doesn't exist
        if (m_trajectories.find(actorID) == m_trajectories.end())
        {
            Trajectory traj;
            traj.actorID = actorID;
            traj.color = XMFLOAT3(
                0.2f + (actorID % 100) / 100.0f * 0.8f,
                0.2f + ((actorID * 13) % 100) / 100.0f * 0.8f,
                0.2f + ((actorID * 37) % 100) / 100.0f * 0.8f
            );
            m_trajectories[actorID] = traj;
        }

        m_trajectories[actorID].points.push_back(point);

        // Remove old points
        auto& points = m_trajectories[actorID].points;
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [this, currentTime](const TrajectoryPoint& tp) {
                    return (currentTime - tp.timeStamp) > m_trajectoryMaxDuration;
                }),
            points.end()
        );
    }
}

void DX12Renderer::RenderTrajectories(UINT& objectIndex)
{
    // PERFORMANCE OPTIMIZATION: Only update visualization every N frames
    static int trajectoryFrameCounter = 0;
    const int VISUALIZATION_UPDATE_INTERVAL = RenderConstants::kVisualizationUpdateInterval;
    trajectoryFrameCounter++;
    if (trajectoryFrameCounter % VISUALIZATION_UPDATE_INTERVAL != 0)
    {
        return;  // Skip this frame
    }

    // Build filtered list of trajectories based on filter mode
    std::vector<const Trajectory*> filteredTrajectories;

    switch (m_trajectoryFilterMode)
    {
    case TrajectoryFilterMode::ALL_ACTORS:
        // Include all trajectories
        for (const auto& pair : m_trajectories)
        {
            if (pair.second.points.size() >= 2)
                filteredTrajectories.push_back(&pair.second);
        }
        break;

    case TrajectoryFilterMode::SELECTED_ONLY:
        // Include only selected actors
        for (const auto& pair : m_trajectories)
        {
            if (pair.second.points.size() >= 2 &&
                m_selectedTrajectoryActors.find(pair.first) != m_selectedTrajectoryActors.end())
            {
                filteredTrajectories.push_back(&pair.second);
            }
        }
        break;

    case TrajectoryFilterMode::FASTEST_N:
    {
        // Calculate speed for each trajectory and sort by speed
        std::vector<std::pair<float, const Trajectory*>> trajWithSpeed;
        for (const auto& pair : m_trajectories)
        {
            if (pair.second.points.size() >= 2)
            {
                // Calculate average speed from recent points
                float totalDist = 0.0f;
                float totalTime = 0.0f;
                const auto& pts = pair.second.points;
                size_t startIdx = pts.size() > 5 ? pts.size() - 5 : 0;
                for (size_t i = startIdx; i < pts.size() - 1; i++)
                {
                    float dx = pts[i + 1].position.x - pts[i].position.x;
                    float dy = pts[i + 1].position.y - pts[i].position.y;
                    float dz = pts[i + 1].position.z - pts[i].position.z;
                    totalDist += sqrtf(dx * dx + dy * dy + dz * dz);
                    totalTime += pts[i + 1].timeStamp - pts[i].timeStamp;
                }
                float speed = totalTime > 0.0f ? totalDist / totalTime : 0.0f;
                trajWithSpeed.push_back({ speed, &pair.second });
            }
        }
        // Sort by speed (descending)
        std::sort(trajWithSpeed.begin(), trajWithSpeed.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        // Take top N (parentheses around std::min to avoid Windows.h macro conflict)
        int count = (std::min)(m_trajectoryMaxCount, static_cast<int>(trajWithSpeed.size()));
        for (int i = 0; i < count; i++)
        {
            filteredTrajectories.push_back(trajWithSpeed[i].second);
        }
        break;
    }

    case TrajectoryFilterMode::MOST_RECENT_N:
    {
        // Sort by most recent point timestamp
        std::vector<std::pair<float, const Trajectory*>> trajWithTime;
        for (const auto& pair : m_trajectories)
        {
            if (pair.second.points.size() >= 2)
            {
                float mostRecentTime = pair.second.points.back().timeStamp;
                trajWithTime.push_back({ mostRecentTime, &pair.second });
            }
        }
        // Sort by time (descending - most recent first)
        std::sort(trajWithTime.begin(), trajWithTime.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        // Take top N
        int count = (std::min)(m_trajectoryMaxCount, static_cast<int>(trajWithTime.size()));
        for (int i = 0; i < count; i++)
        {
            filteredTrajectories.push_back(trajWithTime[i].second);
        }
        break;
    }
    }

    // PERFORMANCE OPTIMIZATION: Limit the number of trajectory segments
    const int MAX_TRAJECTORY_SEGMENTS = RenderConstants::kMaxTrajectorySegments;
    int segmentCount = 0;

    for (const Trajectory* traj : filteredTrajectories)
    {
        // Render trajectory as line segments
        for (size_t i = 0; i < traj->points.size() - 1; i++)
        {
            if (segmentCount >= MAX_TRAJECTORY_SEGMENTS) return;  // Stop rendering all trajectories
            XMFLOAT3 start = traj->points[i].position;
            XMFLOAT3 end = traj->points[i + 1].position;

            auto lineMesh = std::make_unique<RenderMesh>(CreateLineMesh(start, end, traj->color));

            // Check if we're exceeding constant buffer capacity
            if (objectIndex >= RenderConstants::kMaxObjects)
            {
                return;  // Skip rendering remaining trajectory segments
            }

            XMMATRIX world = XMMatrixIdentity();
            UpdateConstantBuffer(world, objectIndex);

            // Bind constant buffer with offset for this specific object
            D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
            commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

            commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
            commandList->IASetVertexBuffers(0, 1, &lineMesh->vertexBufferView);
            commandList->IASetIndexBuffer(&lineMesh->indexBufferView);
            commandList->DrawIndexedInstanced(lineMesh->indexCount, 1, 0, 0, 0);

            commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            dynamicMeshCache[frameIndex].push_back(std::move(lineMesh));
            objectIndex++;
            segmentCount++;
        }
    }
}


//=============================================================================
// Line Mesh Creation
//=============================================================================

RenderMesh DX12Renderer::CreateLineMesh(const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color)
{
    RenderMesh mesh;
    mesh.color = color;

    // Create line vertices
    Vertex vertices[2];
    vertices[0].position = start;
    vertices[0].normal = XMFLOAT3(0, 1, 0);
    vertices[0].color = color;

    vertices[1].position = end;
    vertices[1].normal = XMFLOAT3(0, 1, 0);
    vertices[1].color = color;

    // Create indices
    UINT indices[2] = { 0, 1 };

    // Create vertex buffer
    const UINT vertexBufferSize = sizeof(vertices);

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    // Copy vertex data
    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices, vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    // Initialize vertex buffer view
    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = sizeof(indices);

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    // Copy index data
    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices, indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    // Initialize index buffer view
    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    mesh.indexCount = 2;

    return mesh;
}

RenderMesh DX12Renderer::CreateArrowMesh(const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color,
                                          float headRadius, float headLengthRatio)
{
    RenderMesh mesh;
    mesh.color = color;

    // Calculate arrow direction and length
    XMVECTOR vStart = XMLoadFloat3(&start);
    XMVECTOR vEnd = XMLoadFloat3(&end);
    XMVECTOR vDir = XMVectorSubtract(vEnd, vStart);
    float length = XMVectorGetX(XMVector3Length(vDir));

    if (length < 0.001f)
    {
        // Too short, just return a simple line
        return CreateLineMesh(start, end, color);
    }

    vDir = XMVector3Normalize(vDir);

    // Arrow head size based on length
    float headLength = length * headLengthRatio;
    float actualHeadRadius = headRadius * (length / 2.0f);  // Scale with length

    // Calculate the point where the cone base starts
    XMVECTOR vConeBase = XMVectorSubtract(vEnd, XMVectorScale(vDir, headLength));
    XMFLOAT3 coneBase;
    XMStoreFloat3(&coneBase, vConeBase);

    // Create perpendicular vectors for cone base
    XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    if (fabsf(XMVectorGetY(vDir)) > 0.99f)
    {
        up = XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f);
    }

    XMVECTOR vRight = XMVector3Normalize(XMVector3Cross(vDir, up));
    XMVECTOR vUp = XMVector3Normalize(XMVector3Cross(vRight, vDir));

    // Create vertices: 2 for line + cone vertices
    const int CONE_SEGMENTS = 8;
    const int VERTEX_COUNT = 2 + 1 + CONE_SEGMENTS;  // line vertices + tip + base vertices

    std::vector<Vertex> vertices(VERTEX_COUNT);

    // Line vertices (shaft)
    vertices[0].position = start;
    vertices[0].normal = XMFLOAT3(0, 1, 0);
    vertices[0].color = color;

    vertices[1].position = coneBase;
    vertices[1].normal = XMFLOAT3(0, 1, 0);
    vertices[1].color = color;

    // Cone tip
    vertices[2].position = end;
    XMFLOAT3 tipNormal;
    XMStoreFloat3(&tipNormal, vDir);
    vertices[2].normal = tipNormal;
    vertices[2].color = color;

    // Cone base vertices
    for (int i = 0; i < CONE_SEGMENTS; i++)
    {
        float angle = (2.0f * XM_PI * i) / CONE_SEGMENTS;
        XMVECTOR offset = XMVectorAdd(
            XMVectorScale(vRight, cosf(angle) * actualHeadRadius),
            XMVectorScale(vUp, sinf(angle) * actualHeadRadius)
        );
        XMVECTOR pos = XMVectorAdd(vConeBase, offset);

        XMFLOAT3 basePos;
        XMStoreFloat3(&basePos, pos);
        vertices[3 + i].position = basePos;

        // Normal pointing outward
        XMVECTOR normal = XMVector3Normalize(XMVectorAdd(offset, XMVectorScale(vDir, 0.5f)));
        XMFLOAT3 normalFloat;
        XMStoreFloat3(&normalFloat, normal);
        vertices[3 + i].normal = normalFloat;
        vertices[3 + i].color = color;
    }

    // Create indices
    // Line: 2 indices
    // Cone: CONE_SEGMENTS triangles (3 indices each)
    const int INDEX_COUNT = 2 + CONE_SEGMENTS * 3;
    std::vector<UINT> indices(INDEX_COUNT);

    // Line indices
    indices[0] = 0;
    indices[1] = 1;

    // Cone indices (triangles from tip to base)
    for (int i = 0; i < CONE_SEGMENTS; i++)
    {
        int baseIdx = 2 + i * 3;
        indices[baseIdx] = 2;  // Tip
        indices[baseIdx + 1] = 3 + i;  // Current base vertex
        indices[baseIdx + 2] = 3 + ((i + 1) % CONE_SEGMENTS);  // Next base vertex
    }

    // Create vertex buffer
    const UINT vertexBufferSize = static_cast<UINT>(vertices.size() * sizeof(Vertex));

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.vertexBuffer)
    ));

    UINT8* pVertexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    ThrowIfFailed(mesh.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pVertexDataBegin)));
    memcpy(pVertexDataBegin, vertices.data(), vertexBufferSize);
    mesh.vertexBuffer->Unmap(0, nullptr);

    mesh.vertexBufferView.BufferLocation = mesh.vertexBuffer->GetGPUVirtualAddress();
    mesh.vertexBufferView.StrideInBytes = sizeof(Vertex);
    mesh.vertexBufferView.SizeInBytes = vertexBufferSize;

    // Create index buffer
    const UINT indexBufferSize = static_cast<UINT>(indices.size() * sizeof(UINT));

    ThrowIfFailed(device->CreateCommittedResource(
        &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
        D3D12_HEAP_FLAG_NONE,
        &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
        D3D12_RESOURCE_STATE_GENERIC_READ,
        nullptr,
        IID_PPV_ARGS(&mesh.indexBuffer)
    ));

    UINT8* pIndexDataBegin;
    ThrowIfFailed(mesh.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
    memcpy(pIndexDataBegin, indices.data(), indexBufferSize);
    mesh.indexBuffer->Unmap(0, nullptr);

    mesh.indexBufferView.BufferLocation = mesh.indexBuffer->GetGPUVirtualAddress();
    mesh.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
    mesh.indexBufferView.SizeInBytes = indexBufferSize;

    mesh.indexCount = INDEX_COUNT;

    return mesh;
}


//=============================================================================
// Line Mesh Pooling Implementation
//=============================================================================

void DX12Renderer::InitializeLineMeshPool(UINT poolSize)
{
    // Initialize line mesh pool (2 vertices, 2 indices each)
    m_lineMeshPool.resize(poolSize);

    for (UINT i = 0; i < poolSize; i++)
    {
        PooledMesh& pm = m_lineMeshPool[i];
        pm.maxVertexCount = 2;
        pm.maxIndexCount = 2;

        // Create vertex buffer (kept mapped for fast updates)
        const UINT vertexBufferSize = pm.maxVertexCount * sizeof(Vertex);
        ThrowIfFailed(device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
            D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&pm.vertexBuffer)
        ));

        // Map and keep mapped for fast updates
        CD3DX12_RANGE readRange(0, 0);
        ThrowIfFailed(pm.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pm.vertexDataPtr)));

        pm.vertexBufferView.BufferLocation = pm.vertexBuffer->GetGPUVirtualAddress();
        pm.vertexBufferView.StrideInBytes = sizeof(Vertex);
        pm.vertexBufferView.SizeInBytes = vertexBufferSize;

        // Create index buffer with fixed indices
        const UINT indexBufferSize = pm.maxIndexCount * sizeof(UINT);
        ThrowIfFailed(device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
            D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&pm.indexBuffer)
        ));

        UINT8* pIndexDataBegin;
        ThrowIfFailed(pm.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin)));
        UINT indices[2] = { 0, 1 };
        memcpy(pIndexDataBegin, indices, indexBufferSize);
        pm.indexBuffer->Unmap(0, nullptr);

        pm.indexBufferView.BufferLocation = pm.indexBuffer->GetGPUVirtualAddress();
        pm.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
        pm.indexBufferView.SizeInBytes = indexBufferSize;
    }

    // Initialize arrow mesh pool (up to 12 vertices, up to 26 indices each)
    m_arrowMeshPool.resize(ARROW_MESH_POOL_SIZE);

    for (UINT i = 0; i < ARROW_MESH_POOL_SIZE; i++)
    {
        PooledMesh& pm = m_arrowMeshPool[i];
        pm.maxVertexCount = ARROW_MAX_VERTICES;
        pm.maxIndexCount = ARROW_MAX_INDICES;

        // Create vertex buffer
        const UINT vertexBufferSize = pm.maxVertexCount * sizeof(Vertex);
        ThrowIfFailed(device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
            D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(vertexBufferSize),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&pm.vertexBuffer)
        ));

        CD3DX12_RANGE readRange(0, 0);
        ThrowIfFailed(pm.vertexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pm.vertexDataPtr)));

        pm.vertexBufferView.BufferLocation = pm.vertexBuffer->GetGPUVirtualAddress();
        pm.vertexBufferView.StrideInBytes = sizeof(Vertex);
        pm.vertexBufferView.SizeInBytes = vertexBufferSize;

        // Create index buffer
        const UINT indexBufferSize = pm.maxIndexCount * sizeof(UINT);
        ThrowIfFailed(device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
            D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(indexBufferSize),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&pm.indexBuffer)
        ));

        pm.indexBufferView.BufferLocation = pm.indexBuffer->GetGPUVirtualAddress();
        pm.indexBufferView.Format = DXGI_FORMAT_R32_UINT;
        pm.indexBufferView.SizeInBytes = indexBufferSize;
    }

    m_lineMeshPoolIndex = 0;
    m_arrowMeshPoolIndex = 0;
}

RenderMesh* DX12Renderer::GetPooledLineMesh()
{
    if (m_lineMeshPoolIndex >= m_lineMeshPool.size())
    {
        return nullptr;  // Pool exhausted
    }

    PooledMesh& pm = m_lineMeshPool[m_lineMeshPoolIndex++];

    // Create a temporary RenderMesh referencing the pooled resources
    // The caller should NOT store this long-term as it references pooled resources
    static thread_local RenderMesh tempMesh;
    tempMesh.vertexBuffer = pm.vertexBuffer;
    tempMesh.indexBuffer = pm.indexBuffer;
    tempMesh.vertexBufferView = pm.vertexBufferView;
    tempMesh.indexBufferView = pm.indexBufferView;
    tempMesh.indexCount = 2;

    return &tempMesh;
}

RenderMesh* DX12Renderer::GetPooledArrowMesh()
{
    if (m_arrowMeshPoolIndex >= m_arrowMeshPool.size())
    {
        return nullptr;  // Pool exhausted
    }

    PooledMesh& pm = m_arrowMeshPool[m_arrowMeshPoolIndex++];

    static thread_local RenderMesh tempMesh;
    tempMesh.vertexBuffer = pm.vertexBuffer;
    tempMesh.indexBuffer = pm.indexBuffer;
    tempMesh.vertexBufferView = pm.vertexBufferView;
    tempMesh.indexBufferView = pm.indexBufferView;
    // indexCount will be set by UpdatePooledArrowData

    return &tempMesh;
}

void DX12Renderer::ResetLineMeshPool()
{
    m_lineMeshPoolIndex = 0;
    m_arrowMeshPoolIndex = 0;
}

void DX12Renderer::UpdatePooledMeshData(RenderMesh* mesh, const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color)
{
    if (!mesh || m_lineMeshPoolIndex == 0) return;

    // Get the actual pooled mesh (the one we just allocated)
    PooledMesh& pm = m_lineMeshPool[m_lineMeshPoolIndex - 1];

    Vertex vertices[2];
    vertices[0].position = start;
    vertices[0].normal = XMFLOAT3(0, 1, 0);
    vertices[0].color = color;

    vertices[1].position = end;
    vertices[1].normal = XMFLOAT3(0, 1, 0);
    vertices[1].color = color;

    // Update vertex data directly through mapped pointer
    memcpy(pm.vertexDataPtr, vertices, sizeof(vertices));

    mesh->color = color;
}

void DX12Renderer::UpdatePooledArrowData(RenderMesh* mesh, const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color,
                                          float headRadius, float headLengthRatio)
{
    if (!mesh || m_arrowMeshPoolIndex == 0) return;

    PooledMesh& pm = m_arrowMeshPool[m_arrowMeshPoolIndex - 1];

    // Calculate arrow direction and length
    XMVECTOR vStart = XMLoadFloat3(&start);
    XMVECTOR vEnd = XMLoadFloat3(&end);
    XMVECTOR vDir = XMVectorSubtract(vEnd, vStart);
    float length = XMVectorGetX(XMVector3Length(vDir));

    if (length < 0.001f)
    {
        // Too short - just draw a line
        Vertex vertices[2];
        vertices[0].position = start;
        vertices[0].normal = XMFLOAT3(0, 1, 0);
        vertices[0].color = color;
        vertices[1].position = end;
        vertices[1].normal = XMFLOAT3(0, 1, 0);
        vertices[1].color = color;

        memcpy(pm.vertexDataPtr, vertices, sizeof(vertices));

        // Update indices for just a line
        UINT indices[2] = { 0, 1 };
        UINT8* pIndexDataBegin;
        CD3DX12_RANGE readRange(0, 0);
        pm.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin));
        memcpy(pIndexDataBegin, indices, sizeof(indices));
        pm.indexBuffer->Unmap(0, nullptr);

        mesh->indexCount = 2;
        mesh->color = color;
        return;
    }

    vDir = XMVector3Normalize(vDir);

    // Arrow head size
    float headLength = length * headLengthRatio;
    float actualHeadRadius = headRadius * (length / 2.0f);

    // Calculate cone base position
    XMVECTOR vConeBase = XMVectorSubtract(vEnd, XMVectorScale(vDir, headLength));
    XMFLOAT3 coneBase;
    XMStoreFloat3(&coneBase, vConeBase);

    // Create perpendicular vectors
    XMVECTOR up = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    if (fabsf(XMVectorGetY(vDir)) > 0.99f)
    {
        up = XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f);
    }
    XMVECTOR vRight = XMVector3Normalize(XMVector3Cross(vDir, up));
    XMVECTOR vUp = XMVector3Normalize(XMVector3Cross(vRight, vDir));

    // Build vertices
    const int CONE_SEGMENTS = 8;
    Vertex vertices[ARROW_MAX_VERTICES];

    // Line vertices
    vertices[0].position = start;
    vertices[0].normal = XMFLOAT3(0, 1, 0);
    vertices[0].color = color;

    vertices[1].position = coneBase;
    vertices[1].normal = XMFLOAT3(0, 1, 0);
    vertices[1].color = color;

    // Cone tip
    vertices[2].position = end;
    vertices[2].normal = XMFLOAT3(0, 1, 0);
    vertices[2].color = color;

    // Cone base vertices
    for (int i = 0; i < CONE_SEGMENTS; i++)
    {
        float angle = (2.0f * XM_PI * i) / CONE_SEGMENTS;
        XMVECTOR offset = XMVectorAdd(
            XMVectorScale(vRight, cosf(angle) * actualHeadRadius),
            XMVectorScale(vUp, sinf(angle) * actualHeadRadius)
        );
        XMVECTOR baseVertex = XMVectorAdd(vConeBase, offset);

        XMStoreFloat3(&vertices[3 + i].position, baseVertex);
        vertices[3 + i].normal = XMFLOAT3(0, 1, 0);
        vertices[3 + i].color = color;
    }

    memcpy(pm.vertexDataPtr, vertices, (3 + CONE_SEGMENTS) * sizeof(Vertex));

    // Build indices
    const int INDEX_COUNT = 2 + CONE_SEGMENTS * 3;
    UINT indices[ARROW_MAX_INDICES];
    indices[0] = 0;
    indices[1] = 1;

    for (int i = 0; i < CONE_SEGMENTS; i++)
    {
        int baseIdx = 2 + i * 3;
        indices[baseIdx] = 2;
        indices[baseIdx + 1] = 3 + i;
        indices[baseIdx + 2] = 3 + ((i + 1) % CONE_SEGMENTS);
    }

    UINT8* pIndexDataBegin;
    CD3DX12_RANGE readRange(0, 0);
    pm.indexBuffer->Map(0, &readRange, reinterpret_cast<void**>(&pIndexDataBegin));
    memcpy(pIndexDataBegin, indices, INDEX_COUNT * sizeof(UINT));
    pm.indexBuffer->Unmap(0, nullptr);

    mesh->indexCount = INDEX_COUNT;
    mesh->color = color;
}


//=============================================================================
// Heatmap Visualization Implementation
//=============================================================================

XMFLOAT3 DX12Renderer::CalculateHeatmapColor(float value, float minVal, float maxVal)
{
    // Clamp value to range
    if (value < minVal) value = minVal;
    if (value > maxVal) value = maxVal;

    // Normalize to 0-1
    float normalized = (value - minVal) / (maxVal - minVal);

    // Create heat map gradient: Blue -> Cyan -> Green -> Yellow -> Red
    XMFLOAT3 color;

    if (normalized < 0.25f)
    {
        // Blue to Cyan
        float t = normalized / 0.25f;
        color = XMFLOAT3(0.0f, t, 1.0f);
    }
    else if (normalized < 0.5f)
    {
        // Cyan to Green
        float t = (normalized - 0.25f) / 0.25f;
        color = XMFLOAT3(0.0f, 1.0f, 1.0f - t);
    }
    else if (normalized < 0.75f)
    {
        // Green to Yellow
        float t = (normalized - 0.5f) / 0.25f;
        color = XMFLOAT3(t, 1.0f, 0.0f);
    }
    else
    {
        // Yellow to Red
        float t = (normalized - 0.75f) / 0.25f;
        color = XMFLOAT3(1.0f, 1.0f - t, 0.0f);
    }

    return color;
}

XMFLOAT3 DX12Renderer::GetHeatmapColorForActor(PxRigidActor* actor)
{
    if (!actor) return XMFLOAT3(0.5f, 0.5f, 0.5f);

    // Only dynamic actors have velocity and kinetic energy
    PxRigidDynamic* dynamicActor = actor->is<PxRigidDynamic>();
    if (!dynamicActor)
    {
        // Static actors are rendered in gray
        return XMFLOAT3(0.3f, 0.3f, 0.3f);
    }

    float value = 0.0f;

    switch (m_heatmapMode)
    {
    case HeatmapMode::VELOCITY:
    {
        PxVec3 velocity = dynamicActor->getLinearVelocity();
        value = velocity.magnitude();
        break;
    }
    case HeatmapMode::KINETIC_ENERGY:
    {
        PxVec3 velocity = dynamicActor->getLinearVelocity();
        float mass = dynamicActor->getMass();
        float speed = velocity.magnitude();
        value = 0.5f * mass * speed * speed;  // KE = 0.5 * m * v^2
        break;
    }
    default:
        return XMFLOAT3(0.5f, 0.5f, 0.5f);
    }

    return CalculateHeatmapColor(value, m_heatmapMin, m_heatmapMax);
}

void DX12Renderer::UpdateHeatmapRange(PxScene* scene)
{
    if (!m_heatmapAutoScale || m_heatmapMode == HeatmapMode::NONE || !scene)
        return;

    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
    if (nbActors == 0) return;

    std::vector<PxRigidActor*> actors(nbActors);
    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC,
                     reinterpret_cast<PxActor**>(&actors[0]), nbActors);

    float minVal = FLT_MAX;
    float maxVal = 0.0f;

    for (PxU32 i = 0; i < nbActors; i++)
    {
        PxRigidDynamic* dynamicActor = actors[i]->is<PxRigidDynamic>();
        if (!dynamicActor) continue;

        float value = 0.0f;
        PxVec3 vel = dynamicActor->getLinearVelocity();

        if (m_heatmapMode == HeatmapMode::VELOCITY)
        {
            value = vel.magnitude();
        }
        else if (m_heatmapMode == HeatmapMode::KINETIC_ENERGY)
        {
            float mass = dynamicActor->getMass();
            value = 0.5f * mass * vel.magnitudeSquared();
        }

        minVal = (std::min)(minVal, value);
        maxVal = (std::max)(maxVal, value);
    }

    if (minVal == FLT_MAX) minVal = 0.0f;

    // Smooth transition to avoid flickering
    const float smoothFactor = 0.1f;
    m_heatmapSmoothMin = m_heatmapSmoothMin * (1.0f - smoothFactor) + minVal * smoothFactor;
    m_heatmapSmoothMax = m_heatmapSmoothMax * (1.0f - smoothFactor) + maxVal * smoothFactor;

    m_heatmapMin = m_heatmapSmoothMin;
    m_heatmapMax = (std::max)(m_heatmapSmoothMax, m_heatmapMin + 0.1f);
}

int DX12Renderer::CalculateVisualizationInterval(float currentFPS)
{
    if (!m_adaptiveVisualization)
        return RenderConstants::kVisualizationUpdateInterval;

    // If FPS is high, update visualizations more frequently
    // If FPS is low, reduce visualization updates
    int interval = static_cast<int>(currentFPS / m_targetVisualizationFPS);
    return std::clamp(interval, 1, 30);
}

void DX12Renderer::PerformanceHistory::AddSample(float frameTime, float physicsTime, float renderTime, float fps)
{
    // Add sample and maintain max size
    frameTimeHistory.push_back(frameTime);
    physicsTimeHistory.push_back(physicsTime);
    renderTimeHistory.push_back(renderTime);
    fpsHistory.push_back(fps);

    // Trim to max size
    while (frameTimeHistory.size() > MAX_SAMPLES)
    {
        frameTimeHistory.erase(frameTimeHistory.begin());
        physicsTimeHistory.erase(physicsTimeHistory.begin());
        renderTimeHistory.erase(renderTimeHistory.begin());
        fpsHistory.erase(fpsHistory.begin());
    }
}

void DX12Renderer::PerformanceHistory::Clear()
{
    frameTimeHistory.clear();
    physicsTimeHistory.clear();
    renderTimeHistory.clear();
    fpsHistory.clear();
}

//=============================================================================
// Joint Visualization
//=============================================================================

void DX12Renderer::RenderJoints(PxScene* scene, UINT& objectIndex)
{
    if (!scene) return;

    // Get constraints from the scene
    PxU32 nbConstraints = scene->getNbConstraints();
    if (nbConstraints == 0) return;

    std::vector<PxConstraint*> constraints(nbConstraints);
    scene->getConstraints(constraints.data(), nbConstraints);

    for (PxU32 i = 0; i < nbConstraints; i++)
    {
        PxConstraint* constraint = constraints[i];
        if (!constraint) continue;

        // Get the joint from the constraint
        PxU32 typeID = 0;
        void* externalRef = constraint->getExternalReference(typeID);
        if (typeID != PxConstraintExtIDs::eJOINT || !externalRef) continue;
        PxJoint* joint = reinterpret_cast<PxJoint*>(externalRef);

        // Get the two actors connected by the joint
        PxRigidActor* actor0 = nullptr;
        PxRigidActor* actor1 = nullptr;
        joint->getActors(actor0, actor1);

        // Get local frames
        PxTransform localFrame0 = joint->getLocalPose(PxJointActorIndex::eACTOR0);
        PxTransform localFrame1 = joint->getLocalPose(PxJointActorIndex::eACTOR1);

        // Calculate world positions of joint attachment points
        PxVec3 worldPos0, worldPos1;

        if (actor0)
        {
            PxTransform globalPose0 = actor0->getGlobalPose();
            worldPos0 = globalPose0.transform(localFrame0.p);
        }
        else
        {
            worldPos0 = localFrame0.p; // World anchor
        }

        if (actor1)
        {
            PxTransform globalPose1 = actor1->getGlobalPose();
            worldPos1 = globalPose1.transform(localFrame1.p);
        }
        else
        {
            worldPos1 = localFrame1.p; // World anchor
        }

        // Determine joint color based on type
        XMFLOAT3 jointColor;

        switch (joint->getConcreteType())
        {
        case PxJointConcreteType::eFIXED:
            jointColor = XMFLOAT3(0.8f, 0.8f, 0.8f); // White/gray for fixed
            break;
        case PxJointConcreteType::eREVOLUTE:
            jointColor = XMFLOAT3(1.0f, 0.5f, 0.0f); // Orange for revolute
            break;
        case PxJointConcreteType::eSPHERICAL:
            jointColor = XMFLOAT3(0.0f, 1.0f, 0.5f); // Cyan for spherical
            break;
        case PxJointConcreteType::eDISTANCE:
            jointColor = XMFLOAT3(0.0f, 0.5f, 1.0f); // Blue for distance
            break;
        case PxJointConcreteType::ePRISMATIC:
            jointColor = XMFLOAT3(1.0f, 1.0f, 0.0f); // Yellow for prismatic
            break;
        case PxJointConcreteType::eD6:
            jointColor = XMFLOAT3(1.0f, 0.0f, 1.0f); // Magenta for D6
            break;
        default:
            jointColor = XMFLOAT3(0.5f, 0.5f, 0.5f); // Gray for unknown
            break;
        }

        // Check for broken joints
        if (constraint->getFlags() & PxConstraintFlag::eBROKEN)
        {
            jointColor = XMFLOAT3(1.0f, 0.0f, 0.0f); // Red for broken
        }

        // Render line connecting the two attachment points
        XMFLOAT3 start(worldPos0.x, worldPos0.y, worldPos0.z);
        XMFLOAT3 end(worldPos1.x, worldPos1.y, worldPos1.z);

        // Create line mesh
        auto lineMesh = std::make_unique<RenderMesh>(CreateLineMesh(start, end, jointColor));

        // Check if we're exceeding constant buffer capacity
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;
        }

        XMMATRIX world = XMMatrixIdentity();
        UpdateConstantBuffer(world, objectIndex);

        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_LINELIST);
        commandList->IASetVertexBuffers(0, 1, &lineMesh->vertexBufferView);
        commandList->IASetIndexBuffer(&lineMesh->indexBufferView);
        commandList->DrawIndexedInstanced(lineMesh->indexCount, 1, 0, 0, 0);

        dynamicMeshCache[frameIndex].push_back(std::move(lineMesh));
        objectIndex++;

        // Render small spheres at attachment points
        // TODO: Add small spheres at joint attachment points for better visualization
    }
}

//=============================================================================
// Break Effect Visualization
//=============================================================================

void DX12Renderer::AddBreakEffect(const PxVec3& position)
{
    std::lock_guard<std::mutex> lock(m_breakEffectMutex);

    BreakEffect effect;
    effect.position = XMFLOAT3(position.x, position.y, position.z);
    effect.timeRemaining = m_breakEffectDuration;
    effect.maxTime = m_breakEffectDuration;

    m_breakEffects.push_back(effect);
}

void DX12Renderer::RenderBreakEffects(UINT& objectIndex)
{
    std::lock_guard<std::mutex> lock(m_breakEffectMutex);

    if (m_breakEffects.empty()) return;

    float currentTime = static_cast<float>(ImGui::GetTime());
    static float lastTime = currentTime;
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    // Update and remove expired effects
    m_breakEffects.erase(
        std::remove_if(m_breakEffects.begin(), m_breakEffects.end(),
            [deltaTime](BreakEffect& effect) {
                effect.timeRemaining -= deltaTime;
                return effect.timeRemaining <= 0.0f;
            }),
        m_breakEffects.end()
    );

    // Render each break effect as an expanding ring
    for (const auto& effect : m_breakEffects)
    {
        if (objectIndex >= RenderConstants::kMaxObjects)
        {
            return;
        }

        // Calculate effect parameters based on time
        float progress = 1.0f - (effect.timeRemaining / effect.maxTime);
        float radius = 0.5f + progress * 3.0f;  // Expand from 0.5 to 3.5
        float alpha = 1.0f - progress;  // Fade out

        // Color: orange to red as it expands
        XMFLOAT3 color(1.0f, 0.5f * (1.0f - progress), 0.0f);

        // Create a small sphere at the break position
        PxSphereGeometry sphereGeom(radius * 0.3f);
        RenderMesh mesh = CreateSphereMesh(sphereGeom, color);

        // Position the effect
        XMMATRIX world = XMMatrixTranslation(effect.position.x, effect.position.y, effect.position.z);
        UpdateConstantBuffer(world, objectIndex);

        D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
        commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

        commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
        commandList->IASetIndexBuffer(&mesh.indexBufferView);
        commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);

        objectIndex++;
    }
}

//=============================================================================
// Deformable Volume (Soft Body) Rendering
//=============================================================================

void DX12Renderer::RenderDeformableVolumes(DeformableVolumeManager* deformableManager, UINT& objectIndex)
{
    if (!deformableManager || !deformableManager->IsAvailable()) return;

    const auto& volumes = deformableManager->GetVolumes();

    for (const auto& managed : volumes)
    {
        if (managed.numVertices == 0 || managed.renderPositions.empty()) continue;

        // Render each vertex of the soft body as a small sphere
        // This is a simplified visualization - for production, you'd render the actual tetrahedral surface
        XMFLOAT3 color(managed.colorR, managed.colorG, managed.colorB);

        // Create a small sphere geometry for the vertices
        physx::PxSphereGeometry sphereGeom(0.1f);

        for (physx::PxU32 i = 0; i < managed.numVertices && objectIndex < RenderConstants::kMaxObjects; i++)
        {
            const physx::PxVec4& pos = managed.renderPositions[i];

            // Skip if position seems invalid
            if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) continue;

            // Create mesh for this vertex point
            RenderMesh mesh = CreateSphereMesh(sphereGeom, color);

            // Position the sphere at the vertex location
            XMMATRIX world = XMMatrixTranslation(pos.x, pos.y, pos.z);
            UpdateConstantBuffer(world, objectIndex);

            D3D12_GPU_VIRTUAL_ADDRESS cbAddress = constantBuffer->GetGPUVirtualAddress() + (objectIndex * constantBufferSize);
            commandList->SetGraphicsRootConstantBufferView(0, cbAddress);

            commandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            commandList->IASetVertexBuffers(0, 1, &mesh.vertexBufferView);
            commandList->IASetIndexBuffer(&mesh.indexBufferView);
            commandList->DrawIndexedInstanced(mesh.indexCount, 1, 0, 0, 0);

            objectIndex++;
        }
    }
}

