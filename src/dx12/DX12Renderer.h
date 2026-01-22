#pragma once

#include <d3d12.h>
#include <dxgi1_6.h>
#include <D3Dcompiler.h>
#include <DirectXMath.h>
#include <wrl/client.h>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <mutex>
#include <PxPhysicsAPI.h>

using Microsoft::WRL::ComPtr;

// Note: 'using namespace DirectX' is intentionally kept here because this header
// is specifically for DirectX 12 rendering, and DirectX math types (XMFLOAT3, XMMATRIX, etc.)
// are fundamental throughout. The alternative would make the code significantly less readable.
using namespace DirectX;

//=============================================================================
// RAII Wrapper for Windows HANDLE
//=============================================================================
class ScopedHandle
{
public:
    ScopedHandle() noexcept : m_handle(nullptr) {}
    explicit ScopedHandle(HANDLE handle) noexcept : m_handle(handle) {}
    ~ScopedHandle() noexcept { Close(); }

    // Non-copyable
    ScopedHandle(const ScopedHandle&) = delete;
    ScopedHandle& operator=(const ScopedHandle&) = delete;

    // Movable
    ScopedHandle(ScopedHandle&& other) noexcept : m_handle(other.m_handle) { other.m_handle = nullptr; }
    ScopedHandle& operator=(ScopedHandle&& other) noexcept
    {
        if (this != &other)
        {
            Close();
            m_handle = other.m_handle;
            other.m_handle = nullptr;
        }
        return *this;
    }

    void Close() noexcept
    {
        if (m_handle && m_handle != INVALID_HANDLE_VALUE)
        {
            CloseHandle(m_handle);
            m_handle = nullptr;
        }
    }

    HANDLE Get() const noexcept { return m_handle; }
    HANDLE* GetAddressOf() noexcept { return &m_handle; }
    void Reset(HANDLE handle = nullptr) noexcept { Close(); m_handle = handle; }
    bool IsValid() const noexcept { return m_handle && m_handle != INVALID_HANDLE_VALUE; }
    explicit operator bool() const noexcept { return IsValid(); }

private:
    HANDLE m_handle;
};

//=============================================================================
// Rendering Constants
//=============================================================================
namespace RenderConstants
{
    // Buffer sizes
    static constexpr UINT kMaxObjects = 8192;              // Maximum renderable objects
    static constexpr UINT kConstantBufferAlignment = 256;  // D3D12 constant buffer alignment

    // Default window dimensions
    static constexpr int kDefaultWindowWidth = 1280;
    static constexpr int kDefaultWindowHeight = 720;

    // Sphere tessellation
    static constexpr int kSphereSegments = 16;
    static constexpr int kSphereRings = 16;

    // Visualization settings
    static constexpr int kVisualizationUpdateInterval = 10;  // Update every N frames
    static constexpr int kMaxForceArrows = 50;
    static constexpr int kMaxGravityArrows = 50;
    static constexpr int kMaxVelocityVectors = 100;
    static constexpr int kMaxCollisionPoints = 50;
    static constexpr int kMaxTrajectorySegments = 200;
}

// Forward declarations
class SimulationRecorder;
class PerformanceProfiler;
class ExperimentController;
class DeformableVolumeManager;

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

// Instance data structure for instanced rendering
struct InstanceData
{
    XMMATRIX world;
    XMFLOAT4 color;
};

// Forward declaration
struct RenderMesh;

// Batch data for instanced rendering
struct InstanceBatch
{
    std::shared_ptr<RenderMesh> mesh;
    std::vector<InstanceData> instances;
};

// Camera class
class Camera
{
public:
    Camera();
    void SetPosition(float x, float y, float z);
    void SetLookAt(float x, float y, float z);
    void SetAspectRatio(float aspect) { aspectRatio = aspect; }
    void Update(float deltaTime);
    void HandleInput(WPARAM wParam);
    void HandleKeyUp(WPARAM wParam);
    void HandleMouseOrbit(int dx, int dy);
    void HandleMousePan(int dx, int dy);
    void HandleMouseZoom(float delta);

    XMMATRIX GetViewMatrix() const;
    XMMATRIX GetProjectionMatrix() const;
    XMFLOAT3 GetPosition() const { return position; }
    XMFLOAT3 GetTarget() const { return target; }

private:
    XMFLOAT3 position;
    XMFLOAT3 target;  // Orbit center
    XMFLOAT3 up;
    float distance;   // Distance from target
    float yaw;
    float pitch;
    float moveSpeed;
    float rotateSpeed;
    float panSpeed;
    float zoomSpeed;
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;

    // Key state tracking for smooth movement
    bool keyW, keyS, keyA, keyD, keyQ, keyE;

    // Helper to update position from orbit parameters
    void UpdatePositionFromOrbit();
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
    void RenderActor(physx::PxRigidActor* actor, const XMFLOAT3& color, UINT& objectIndex);
    void RenderPhysXSceneInstanced(physx::PxScene* scene);  // Instanced rendering version
    void RenderDeformableVolumes(DeformableVolumeManager* deformableManager, UINT& objectIndex);  // Soft body rendering

    void SetCamera(const Camera& camera);
    void Resize(UINT width, UINT height);

    Camera& GetCamera() { return camera; }

    // ImGui integration
    void InitializeImGui(HWND hwnd);
    void ShutdownImGui();
    void RenderImGui(physx::PxScene* scene, float deltaTime, physx::PxCudaContextManager* cudaContextManager = nullptr,
                     SimulationRecorder* recorder = nullptr, PerformanceProfiler* profiler = nullptr,
                     ExperimentController* controller = nullptr, physx::PxMaterial* material = nullptr);

    // Visualization options
    void SetVelocityVectorVisualization(bool enable) { m_showVelocityVectors = enable; }
    bool GetVelocityVectorVisualization() const { return m_showVelocityVectors; }
    void SetVelocityScale(float scale) { m_velocityScale = scale; }
    float GetVelocityScale() const { return m_velocityScale; }

    void SetInstancingEnabled(bool enable) { m_useInstancing = enable; }
    bool GetInstancingEnabled() const { return m_useInstancing; }

    void SetCollisionVisualization(bool enable) { m_showCollisions = enable; }
    bool GetCollisionVisualization() const { return m_showCollisions; }
    void AddCollisionPoint(const physx::PxVec3& point, float impulse);
    void ClearCollisionPoints() { m_collisionPoints.clear(); }

    void SetTrajectoryVisualization(bool enable) { m_showTrajectories = enable; }
    bool GetTrajectoryVisualization() const { return m_showTrajectories; }
    void UpdateTrajectories(physx::PxScene* scene, float deltaTime);
    void ClearTrajectories() { m_trajectories.clear(); }

    void SetForceVisualization(bool enable) { m_showForces = enable; }
    bool GetForceVisualization() const { return m_showForces; }
    void SetForceScale(float scale) { m_forceScale = scale; }
    float GetForceScale() const { return m_forceScale; }
    void AddContactForce(const physx::PxVec3& position, const physx::PxVec3& force);
    void ClearContactForces() { m_contactForces.clear(); }

    // Heatmap visualization
    enum class HeatmapMode
    {
        NONE,
        VELOCITY,
        KINETIC_ENERGY
    };

    void SetHeatmapMode(HeatmapMode mode) { m_heatmapMode = mode; }
    HeatmapMode GetHeatmapMode() const { return m_heatmapMode; }
    void SetHeatmapRange(float minVal, float maxVal) { m_heatmapMin = minVal; m_heatmapMax = maxVal; }
    void GetHeatmapRange(float& minVal, float& maxVal) const { minVal = m_heatmapMin; maxVal = m_heatmapMax; }
    void SetHeatmapAutoScale(bool enable) { m_heatmapAutoScale = enable; }
    bool GetHeatmapAutoScale() const { return m_heatmapAutoScale; }

    // Trajectory filtering
    enum class TrajectoryFilterMode
    {
        ALL_ACTORS,
        SELECTED_ONLY,
        FASTEST_N,
        MOST_RECENT_N
    };

    void SetTrajectoryFilterMode(TrajectoryFilterMode mode) { m_trajectoryFilterMode = mode; }
    TrajectoryFilterMode GetTrajectoryFilterMode() const { return m_trajectoryFilterMode; }
    void SetTrajectoryMaxCount(int count) { m_trajectoryMaxCount = count; }
    int GetTrajectoryMaxCount() const { return m_trajectoryMaxCount; }
    void SetTrajectoryDuration(float duration) { m_trajectoryMaxDuration = duration; }
    float GetTrajectoryDuration() const { return m_trajectoryMaxDuration; }

    // Contact normal visualization
    void SetContactNormalVisualization(bool enable) { m_showContactNormals = enable; }
    bool GetContactNormalVisualization() const { return m_showContactNormals; }
    void SetContactNormalScale(float scale) { m_contactNormalScale = scale; }
    void AddContactForceWithNormal(const physx::PxVec3& position, const physx::PxVec3& force, const physx::PxVec3& normal);

    // Joint visualization
    void SetJointVisualization(bool enable) { m_showJoints = enable; }
    bool GetJointVisualization() const { return m_showJoints; }
    void RenderJoints(physx::PxScene* scene, UINT& objectIndex);

    // Break effect visualization
    void AddBreakEffect(const physx::PxVec3& position);
    void RenderBreakEffects(UINT& objectIndex);
    void ClearBreakEffects() { m_breakEffects.clear(); }

    // Visualization thresholds (user-configurable)
    struct VisualizationThresholds
    {
        float velocitySlow = 5.0f;
        float velocityFast = 15.0f;
        float forceSmall = 100.0f;
        float forceLarge = 500.0f;
        float impulseLow = 10.0f;
        float impulseHigh = 50.0f;
    };

    VisualizationThresholds& GetThresholds() { return m_thresholds; }
    const VisualizationThresholds& GetThresholds() const { return m_thresholds; }

    // Adaptive visualization
    void SetAdaptiveVisualization(bool enable) { m_adaptiveVisualization = enable; }
    bool GetAdaptiveVisualization() const { return m_adaptiveVisualization; }

    // Performance history for graphs
    struct PerformanceHistory
    {
        static constexpr size_t MAX_SAMPLES = 300;
        std::vector<float> frameTimeHistory;
        std::vector<float> physicsTimeHistory;
        std::vector<float> renderTimeHistory;
        std::vector<float> fpsHistory;

        void AddSample(float frameTime, float physicsTime, float renderTime, float fps);
        void Clear();
    };

    PerformanceHistory& GetPerformanceHistory() { return m_perfHistory; }

    // Screenshot and video capture
    bool CaptureScreenshot(const std::string& filename);
    void EnableSequentialCapture(bool enable) { m_sequentialCaptureEnabled = enable; }
    bool IsSequentialCaptureEnabled() const { return m_sequentialCaptureEnabled; }
    void SetCaptureInterval(int interval) { m_captureInterval = interval; }  // Capture every N frames
    int GetCaptureFrameCount() const { return m_captureFrameCount; }

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
    RenderMesh CreateCapsuleMesh(const physx::PxCapsuleGeometry& geometry, const XMFLOAT3& color);
    RenderMesh CreateConvexMesh(const physx::PxConvexMeshGeometry& geometry, const XMFLOAT3& color);
    RenderMesh CreateTriangleMesh(const physx::PxTriangleMeshGeometry& geometry, const XMFLOAT3& color);
    RenderMesh CreateHeightfieldMesh(const physx::PxHeightFieldGeometry& geometry, const XMFLOAT3& color);

    // Mesh caching helpers
    std::string GenerateBoxCacheKey(const physx::PxBoxGeometry& geometry);
    std::string GenerateSphereCacheKey(const physx::PxSphereGeometry& geometry);
    std::shared_ptr<RenderMesh> GetOrCreateBoxMesh(const physx::PxBoxGeometry& geometry, const XMFLOAT3& color);
    std::shared_ptr<RenderMesh> GetOrCreateSphereMesh(const physx::PxSphereGeometry& geometry, const XMFLOAT3& color);

    void UpdateConstantBuffer(const XMMATRIX& world, UINT objectIndex = 0);

    // UI styling
    void ApplyModernStyle();

    // Velocity vector visualization
    void RenderVelocityVectors(physx::PxScene* scene, UINT& objectIndex);
    RenderMesh CreateLineMesh(const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color);
    RenderMesh CreateArrowMesh(const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color,
                               float headRadius = 0.15f, float headLength = 0.2f);

    // Line mesh pooling for performance optimization
    void InitializeLineMeshPool(UINT poolSize);
    RenderMesh* GetPooledLineMesh();
    RenderMesh* GetPooledArrowMesh();
    void ResetLineMeshPool();
    void UpdatePooledMeshData(RenderMesh* mesh, const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color);
    void UpdatePooledArrowData(RenderMesh* mesh, const XMFLOAT3& start, const XMFLOAT3& end, const XMFLOAT3& color,
                               float headRadius, float headLength);

    // Contact normal visualization
    void RenderContactNormals(UINT& objectIndex);

    // Heatmap auto-scaling
    void UpdateHeatmapRange(physx::PxScene* scene);

    // Adaptive visualization interval
    int CalculateVisualizationInterval(float currentFPS);

    // Collision visualization
    void RenderCollisionPoints(UINT& objectIndex);

    // Collision data structure
    struct CollisionPoint {
        XMFLOAT3 position;
        float impulse;
        float timeStamp;
    };

    // Trajectory visualization
    void RenderTrajectories(UINT& objectIndex);

    // Force visualization
    void RenderForces(physx::PxScene* scene, UINT& objectIndex);
    void RenderContactForces(UINT& objectIndex);
    void RenderGravityForces(physx::PxScene* scene, UINT& objectIndex);

    // Heatmap visualization helpers
    XMFLOAT3 CalculateHeatmapColor(float value, float minVal, float maxVal);
    XMFLOAT3 GetHeatmapColorForActor(physx::PxRigidActor* actor);

    // Instanced rendering helpers
    void RenderInstanceBatch(const InstanceBatch& batch, UINT& objectIndex);
    void CreateInstanceBuffer(UINT maxInstances);

    // Trajectory data structures
    struct TrajectoryPoint {
        XMFLOAT3 position;
        float timeStamp;
    };

    struct Trajectory {
        uint32_t actorID;
        std::vector<TrajectoryPoint> points;
        XMFLOAT3 color;
    };

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
    UINT constantBufferSize;  // Size of single constant buffer (aligned)

    // Synchronization
    ComPtr<ID3D12Fence> fence;
    UINT64 fenceValues[2];
    ScopedHandle fenceEvent;

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

    // Dock layout initialization flag
    bool m_dockLayoutInitialized = false;

    // Visualization options
    bool m_showVelocityVectors;
    float m_velocityScale;

    bool m_showCollisions;
    std::vector<CollisionPoint> m_collisionPoints;
    mutable std::mutex m_collisionMutex;  // Protects m_collisionPoints
    float m_collisionDisplayDuration;  // How long to show collision points (seconds)

    bool m_showTrajectories;
    std::unordered_map<uint32_t, Trajectory> m_trajectories;  // Actor ID -> Trajectory
    float m_trajectoryUpdateInterval;  // How often to sample positions (seconds)
    float m_trajectoryMaxDuration;  // Max trail length in seconds
    float m_lastTrajectoryUpdate;

    // Force visualization
    bool m_showForces;
    float m_forceScale;  // Scale factor for force arrows

    struct ContactForce {
        XMFLOAT3 position;
        XMFLOAT3 force;
        XMFLOAT3 normal;  // Contact normal direction
        float timeStamp;
    };
    std::vector<ContactForce> m_contactForces;
    mutable std::mutex m_contactForceMutex;  // Protects m_contactForces
    float m_forceDisplayDuration;  // How long to show forces (seconds)

    // Contact normal visualization
    bool m_showContactNormals = false;
    float m_contactNormalScale = 1.0f;

    // Joint visualization
    bool m_showJoints = true;

    // Break effect visualization
    struct BreakEffect
    {
        XMFLOAT3 position;
        float timeRemaining;
        float maxTime;
    };
    std::vector<BreakEffect> m_breakEffects;
    mutable std::mutex m_breakEffectMutex;
    float m_breakEffectDuration = 2.0f;  // How long to show break effects

    // Heatmap visualization
    HeatmapMode m_heatmapMode;
    float m_heatmapMin;  // Minimum value for heatmap range
    float m_heatmapMax;  // Maximum value for heatmap range
    bool m_heatmapAutoScale = false;
    float m_heatmapSmoothMin = 0.0f;
    float m_heatmapSmoothMax = 20.0f;

    // Trajectory filtering
    TrajectoryFilterMode m_trajectoryFilterMode = TrajectoryFilterMode::ALL_ACTORS;
    int m_trajectoryMaxCount = 20;
    std::unordered_set<uint32_t> m_selectedTrajectoryActors;

    // Visualization thresholds
    VisualizationThresholds m_thresholds;

    // Adaptive visualization
    bool m_adaptiveVisualization = true;
    float m_targetVisualizationFPS = 30.0f;

    // Performance history
    PerformanceHistory m_perfHistory;

    // Screenshot and video capture
    bool m_sequentialCaptureEnabled;
    int m_captureInterval;  // Capture every N frames
    int m_captureFrameCount;
    int m_frameCounter;

    // Instanced rendering
    ComPtr<ID3D12Resource> instanceBuffer;
    D3D12_VERTEX_BUFFER_VIEW instanceBufferView;
    UINT8* instanceBufferData;
    UINT maxInstanceCount;
    bool m_useInstancing;  // Toggle instanced rendering

    // Line mesh pool for performance optimization
    // Pre-allocated GPU resources to avoid per-frame allocations
    struct PooledMesh
    {
        ComPtr<ID3D12Resource> vertexBuffer;
        ComPtr<ID3D12Resource> indexBuffer;
        D3D12_VERTEX_BUFFER_VIEW vertexBufferView;
        D3D12_INDEX_BUFFER_VIEW indexBufferView;
        UINT8* vertexDataPtr = nullptr;  // Mapped pointer for fast updates
        UINT maxVertexCount;
        UINT maxIndexCount;
    };
    std::vector<PooledMesh> m_lineMeshPool;
    std::vector<PooledMesh> m_arrowMeshPool;
    UINT m_lineMeshPoolIndex = 0;
    UINT m_arrowMeshPoolIndex = 0;
    static constexpr UINT LINE_MESH_POOL_SIZE = 512;
    static constexpr UINT ARROW_MESH_POOL_SIZE = 256;
    static constexpr UINT ARROW_MAX_VERTICES = 12;  // 2 line + 1 tip + 8 cone base
    static constexpr UINT ARROW_MAX_INDICES = 2 + 24;  // 2 for line + 8*3 for cone
};
