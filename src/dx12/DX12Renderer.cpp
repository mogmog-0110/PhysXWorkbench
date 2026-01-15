#include "DX12Renderer.h"
#include "d3dx12.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"

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
    : position(0.0f, 10.0f, -20.0f)
    , lookAt(0.0f, 0.0f, 0.0f)
    , up(0.0f, 1.0f, 0.0f)
    , yaw(0.0f)
    , pitch(0.0f)
    , moveSpeed(10.0f)
    , rotateSpeed(0.005f)
    , fov(XM_PIDIV4)
    , aspectRatio(16.0f / 9.0f)
    , nearPlane(0.1f)
    , farPlane(1000.0f)
{
}

void Camera::SetPosition(float x, float y, float z)
{
    position = XMFLOAT3(x, y, z);
}

void Camera::SetLookAt(float x, float y, float z)
{
    lookAt = XMFLOAT3(x, y, z);
}

void Camera::Update(float deltaTime)
{
    // Camera is updated via input
}

void Camera::HandleInput(WPARAM wParam)
{
    XMVECTOR forward = XMVectorSubtract(XMLoadFloat3(&lookAt), XMLoadFloat3(&position));
    forward = XMVector3Normalize(forward);
    XMVECTOR right = XMVector3Cross(XMLoadFloat3(&up), forward);
    right = XMVector3Normalize(right);

    XMVECTOR pos = XMLoadFloat3(&position);

    switch (wParam)
    {
    case 'W': // Forward
        pos = XMVectorAdd(pos, XMVectorScale(forward, moveSpeed * 0.1f));
        break;
    case 'S': // Backward
        pos = XMVectorSubtract(pos, XMVectorScale(forward, moveSpeed * 0.1f));
        break;
    case 'A': // Left
        pos = XMVectorAdd(pos, XMVectorScale(right, moveSpeed * 0.1f));
        break;
    case 'D': // Right
        pos = XMVectorSubtract(pos, XMVectorScale(right, moveSpeed * 0.1f));
        break;
    case 'Q': // Up
        pos = XMVectorAdd(pos, XMVectorSet(0, moveSpeed * 0.1f, 0, 0));
        break;
    case 'E': // Down
        pos = XMVectorSubtract(pos, XMVectorSet(0, moveSpeed * 0.1f, 0, 0));
        break;
    }

    XMStoreFloat3(&position, pos);

    // Update lookAt to maintain relative position
    XMVECTOR lookAtVec = XMVectorAdd(pos, forward);
    XMStoreFloat3(&lookAt, lookAtVec);
}

void Camera::HandleMouse(int dx, int dy)
{
    yaw -= dx * rotateSpeed;
    pitch -= dy * rotateSpeed;

    // Clamp pitch to avoid gimbal lock
    const float maxPitch = XM_PI / 2.0f - 0.1f;
    if (pitch > maxPitch) pitch = maxPitch;
    if (pitch < -maxPitch) pitch = -maxPitch;

    // Calculate new forward vector
    float cosPitch = cosf(pitch);
    XMFLOAT3 forward;
    forward.x = sinf(yaw) * cosPitch;
    forward.y = sinf(pitch);
    forward.z = cosf(yaw) * cosPitch;

    XMVECTOR forwardVec = XMLoadFloat3(&forward);
    forwardVec = XMVector3Normalize(forwardVec);

    XMVECTOR pos = XMLoadFloat3(&position);
    XMVECTOR lookAtVec = XMVectorAdd(pos, XMVectorScale(forwardVec, 10.0f));

    XMStoreFloat3(&lookAt, lookAtVec);
}

XMMATRIX Camera::GetViewMatrix() const
{
    return XMMatrixLookAtLH(
        XMLoadFloat3(&position),
        XMLoadFloat3(&lookAt),
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
    , width(1280)
    , height(720)
    , windowHandle(nullptr)
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

        // Create constant buffer
        const UINT constantBufferSize = (sizeof(ConstantBuffer) + 255) & ~255;
        ThrowIfFailed(device->CreateCommittedResource(
            &CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD),
            D3D12_HEAP_FLAG_NONE,
            &CD3DX12_RESOURCE_DESC::Buffer(constantBufferSize),
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&constantBuffer)
        ));

        // Map constant buffer
        CD3DX12_RANGE readRange(0, 0);
        ThrowIfFailed(constantBuffer->Map(0, &readRange, reinterpret_cast<void**>(&constantBufferData)));

        // Create CBV
        D3D12_CONSTANT_BUFFER_VIEW_DESC cbvDesc = {};
        cbvDesc.BufferLocation = constantBuffer->GetGPUVirtualAddress();
        cbvDesc.SizeInBytes = constantBufferSize;
        device->CreateConstantBufferView(&cbvDesc, cbvHeap->GetCPUDescriptorHandleForHeapStart());

        // Create fence for synchronization
        ThrowIfFailed(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&fence)));
        fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (fenceEvent == nullptr)
        {
            ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
        }

        // Initialize camera
        camera.SetPosition(0.0f, 15.0f, -30.0f);
        camera.SetLookAt(0.0f, 5.0f, 0.0f);

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

    CloseHandle(fenceEvent);
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

    // Reset command allocator and list
    ThrowIfFailed(commandAllocators[frameIndex]->Reset());
    ThrowIfFailed(commandList->Reset(commandAllocators[frameIndex].Get(), pipelineState.Get()));

    // Set necessary state
    commandList->SetGraphicsRootSignature(rootSignature.Get());

    ID3D12DescriptorHeap* ppHeaps[] = { cbvHeap.Get() };
    commandList->SetDescriptorHeaps(_countof(ppHeaps), ppHeaps);
    commandList->SetGraphicsRootConstantBufferView(0, constantBuffer->GetGPUVirtualAddress());

    commandList->RSSetViewports(1, &viewport);
    commandList->RSSetScissorRects(1, &scissorRect);

    // Transition render target to render target state
    commandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(renderTargets[frameIndex].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

    // Get render target and depth stencil handles
    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(rtvHeap->GetCPUDescriptorHandleForHeapStart(), frameIndex, rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(dsvHeap->GetCPUDescriptorHandleForHeapStart());

    commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

    // Clear render target and depth stencil
    const float clearColor[] = { 0.1f, 0.1f, 0.15f, 1.0f };
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
            ThrowIfFailed(fence->SetEventOnCompletion(fenceValue, fenceEvent));
            WaitForSingleObject(fenceEvent, INFINITE);
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
        ThrowIfFailed(fence->SetEventOnCompletion(fenceValues[frameIndex], fenceEvent));
        WaitForSingleObject(fenceEvent, INFINITE);
    }

    fenceValues[frameIndex] = currentFenceValue + 1;
}

void DX12Renderer::UpdateConstantBuffer(const XMMATRIX& world)
{
    ConstantBuffer cb;

    XMMATRIX view = camera.GetViewMatrix();
    XMMATRIX proj = camera.GetProjectionMatrix();

    cb.worldViewProj = XMMatrixTranspose(world * view * proj);
    cb.world = XMMatrixTranspose(world);
    cb.lightDir = XMFLOAT4(0.3f, -0.7f, 0.3f, 0.0f);
    cb.cameraPos = XMFLOAT4(camera.GetPosition().x, camera.GetPosition().y, camera.GetPosition().z, 1.0f);

    memcpy(constantBufferData, &cb, sizeof(cb));
}

void DX12Renderer::SetCamera(const Camera& cam)
{
    camera = cam;
}

void DX12Renderer::Resize(UINT width, UINT height)
{
    this->width = width;
    this->height = height;
    // TODO: Implement resize logic
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

        RenderActor(actor, color);
    }
}

void DX12Renderer::RenderActor(PxRigidActor* actor, const XMFLOAT3& color)
{
    if (!actor) return;

    PxU32 nShapes = actor->getNbShapes();
    std::vector<PxShape*> shapes(nShapes);
    actor->getShapes(&shapes[0], nShapes);

    for (PxU32 i = 0; i < nShapes; i++)
    {
        PxShape* shape = shapes[i];

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

        // Update constant buffer with world transform
        UpdateConstantBuffer(world);

        // Get geometry using PxGeometryHolder
        PxGeometryHolder geomHolder = shape->getGeometry();
        PxGeometryType::Enum geomType = geomHolder.getType();

        switch (geomType)
        {
        case PxGeometryType::eBOX:
        {
            const PxBoxGeometry& boxGeom = geomHolder.box();
            auto meshPtr = GetOrCreateBoxMesh(boxGeom, color);

            commandList->IASetVertexBuffers(0, 1, &meshPtr->vertexBufferView);
            commandList->IASetIndexBuffer(&meshPtr->indexBufferView);
            commandList->DrawIndexedInstanced(meshPtr->indexCount, 1, 0, 0, 0);
            break;
        }
        case PxGeometryType::eSPHERE:
        {
            const PxSphereGeometry& sphereGeom = geomHolder.sphere();
            auto meshPtr = GetOrCreateSphereMesh(sphereGeom, color);

            commandList->IASetVertexBuffers(0, 1, &meshPtr->vertexBufferView);
            commandList->IASetIndexBuffer(&meshPtr->indexBufferView);
            commandList->DrawIndexedInstanced(meshPtr->indexCount, 1, 0, 0, 0);
            break;
        }
        case PxGeometryType::ePLANE:
        {
            if (!groundMesh)
            {
                groundMesh = std::make_unique<RenderMesh>(CreatePlaneMesh(color));
            }

            commandList->IASetVertexBuffers(0, 1, &groundMesh->vertexBufferView);
            commandList->IASetIndexBuffer(&groundMesh->indexBufferView);
            commandList->DrawIndexedInstanced(groundMesh->indexCount, 1, 0, 0, 0);
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

    std::cout << "Created and cached new box mesh: " << cacheKey << " (cache size: " << meshCache.size() << ")" << std::endl;

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

    std::cout << "Created and cached new sphere mesh: " << cacheKey << " (cache size: " << meshCache.size() << ")" << std::endl;

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

    const int segments = 16;
    const int rings = 16;
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

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

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

void DX12Renderer::RenderImGui(PxScene* scene, float deltaTime, PxCudaContextManager* cudaContextManager)
{
    // Start the Dear ImGui frame
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    // Debug window
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
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("GPU acceleration is active");
            ImGui::Text("Physics calculations run on CUDA");
            ImGui::EndTooltip();
        }
    }
    else
    {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "DISABLED");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Running on CPU only");
            ImGui::Text("No CUDA-capable GPU detected");
            ImGui::EndTooltip();
        }
    }
    ImGui::Separator();

    // PhysX statistics
    if (scene)
    {
        PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
        PxU32 nbDynamic = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
        PxU32 nbStatic = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC);

        ImGui::Text("Total Actors: %d", nbActors);
        ImGui::Text("  Dynamic: %d", nbDynamic);
        ImGui::Text("  Static: %d", nbStatic);
        ImGui::Separator();

        // Gravity control
        PxVec3 gravity = scene->getGravity();
        float gravityY = gravity.y;
        if (ImGui::SliderFloat("Gravity Y", &gravityY, -20.0f, 0.0f))
        {
            scene->setGravity(PxVec3(gravity.x, gravityY, gravity.z));
        }
        ImGui::Separator();
    }

    // Camera information
    XMFLOAT3 camPos = camera.GetPosition();
    ImGui::Text("Camera Position:");
    ImGui::Text("  X: %.2f", camPos.x);
    ImGui::Text("  Y: %.2f", camPos.y);
    ImGui::Text("  Z: %.2f", camPos.z);
    ImGui::Separator();

    // Mesh cache statistics
    ImGui::Text("Mesh Cache:");
    ImGui::Text("  Cached Meshes: %zu", meshCache.size());
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::Text("Reusable mesh geometries");
        ImGui::Text("Reduces GPU resource creation");
        ImGui::EndTooltip();
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
