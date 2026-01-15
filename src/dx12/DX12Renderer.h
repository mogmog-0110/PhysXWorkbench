#pragma once

#include <d3d12.h>
#include <dxgi1_6.h>
#include <D3Dcompiler.h>
#include <DirectXMath.h>
#include <wrl/client.h>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <PxPhysicsAPI.h>

using Microsoft::WRL::ComPtr;
using namespace DirectX;

// Vertex structure for rendering
struct Vertex
{
    XMFLOAT3 position;
    XMFLOAT3 normal;
    XMFLOAT3 color;
};

// Constant buffer structure
struct ConstantBuffer
{
    XMMATRIX worldViewProj;
    XMMATRIX world;
    XMFLOAT4 lightDir;
    XMFLOAT4 cameraPos;
};

// Camera class
class Camera
{
public:
    Camera();
    void SetPosition(float x, float y, float z);
    void SetLookAt(float x, float y, float z);
    void Update(float deltaTime);
    void HandleInput(WPARAM wParam);
    void HandleMouse(int dx, int dy);

    XMMATRIX GetViewMatrix() const;
    XMMATRIX GetProjectionMatrix() const;
    XMFLOAT3 GetPosition() const { return position; }

private:
    XMFLOAT3 position;
    XMFLOAT3 lookAt;
    XMFLOAT3 up;
    float yaw;
    float pitch;
    float moveSpeed;
    float rotateSpeed;
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;
};

// Mesh representation
struct RenderMesh
{
    ComPtr<ID3D12Resource> vertexBuffer;
    ComPtr<ID3D12Resource> indexBuffer;
    D3D12_VERTEX_BUFFER_VIEW vertexBufferView;
    D3D12_INDEX_BUFFER_VIEW indexBufferView;
    UINT indexCount;
    XMFLOAT3 color;
};

// DirectX 12 Renderer class
class DX12Renderer
{
public:
    DX12Renderer();
    ~DX12Renderer();

    bool Initialize(HWND hwnd, UINT width, UINT height);
    void Shutdown();

    void BeginFrame();
    void EndFrame();

    void RenderPhysXScene(physx::PxScene* scene);
    void RenderActor(physx::PxRigidActor* actor, const XMFLOAT3& color);

    void SetCamera(const Camera& camera);
    void Resize(UINT width, UINT height);

    Camera& GetCamera() { return camera; }

    // ImGui integration
    void InitializeImGui(HWND hwnd);
    void ShutdownImGui();
    void RenderImGui(physx::PxScene* scene, float deltaTime, physx::PxCudaContextManager* cudaContextManager = nullptr);

private:
    // Initialization helpers
    bool CreateDevice();
    bool CreateCommandQueue();
    bool CreateSwapChain(HWND hwnd, UINT width, UINT height);
    bool CreateDescriptorHeaps();
    bool CreateRenderTargets();
    bool CreateDepthStencil();
    bool CreateRootSignature();
    bool CreatePipelineState();
    bool CreateCommandAllocatorsAndLists();
    bool LoadShaders();

    // Rendering helpers
    void WaitForGPU();
    void MoveToNextFrame();

    RenderMesh CreateBoxMesh(const physx::PxBoxGeometry& geometry, const XMFLOAT3& color);
    RenderMesh CreateSphereMesh(const physx::PxSphereGeometry& geometry, const XMFLOAT3& color);
    RenderMesh CreatePlaneMesh(const XMFLOAT3& color);

    // Mesh caching helpers
    std::string GenerateBoxCacheKey(const physx::PxBoxGeometry& geometry);
    std::string GenerateSphereCacheKey(const physx::PxSphereGeometry& geometry);
    std::shared_ptr<RenderMesh> GetOrCreateBoxMesh(const physx::PxBoxGeometry& geometry, const XMFLOAT3& color);
    std::shared_ptr<RenderMesh> GetOrCreateSphereMesh(const physx::PxSphereGeometry& geometry, const XMFLOAT3& color);

    void UpdateConstantBuffer(const XMMATRIX& world);

    // D3D12 objects
    ComPtr<ID3D12Device> device;
    ComPtr<ID3D12CommandQueue> commandQueue;
    ComPtr<IDXGISwapChain3> swapChain;
    ComPtr<ID3D12DescriptorHeap> rtvHeap;
    ComPtr<ID3D12DescriptorHeap> dsvHeap;
    ComPtr<ID3D12DescriptorHeap> cbvHeap;
    ComPtr<ID3D12Resource> renderTargets[2];
    ComPtr<ID3D12Resource> depthStencil;
    ComPtr<ID3D12CommandAllocator> commandAllocators[2];
    ComPtr<ID3D12GraphicsCommandList> commandList;
    ComPtr<ID3D12RootSignature> rootSignature;
    ComPtr<ID3D12PipelineState> pipelineState;

    // Constant buffer
    ComPtr<ID3D12Resource> constantBuffer;
    UINT8* constantBufferData;

    // Synchronization
    ComPtr<ID3D12Fence> fence;
    UINT64 fenceValues[2];
    HANDLE fenceEvent;

    // Frame management
    UINT frameIndex;
    UINT rtvDescriptorSize;

    // Viewport and scissor
    D3D12_VIEWPORT viewport;
    D3D12_RECT scissorRect;

    // Camera
    Camera camera;

    // Cached meshes
    std::unique_ptr<RenderMesh> groundMesh;
    std::vector<std::unique_ptr<RenderMesh>> dynamicMeshCache[2];  // Double buffered

    // Mesh cache system - shared_ptr for reusable meshes
    std::unordered_map<std::string, std::shared_ptr<RenderMesh>> meshCache;

    // Window dimensions
    UINT width;
    UINT height;

    // Shader bytecode
    ComPtr<ID3DBlob> vertexShader;
    ComPtr<ID3DBlob> pixelShader;

    // ImGui descriptor heap
    ComPtr<ID3D12DescriptorHeap> imguiSrvHeap;

    // Window handle for ImGui
    HWND windowHandle;
};
