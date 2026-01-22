#pragma once

#include <PxPhysicsAPI.h>
#include <vector>
#include <memory>

// Managed deformable volume with render data
struct ManagedDeformableVolume
{
    physx::PxDeformableVolume* volume = nullptr;
    physx::PxDeformableVolumeMesh* mesh = nullptr;

    // Render data (updated each frame from GPU)
    std::vector<physx::PxVec4> renderPositions;  // Position buffer for rendering
    std::vector<physx::PxU32> renderIndices;     // Triangle indices
    physx::PxU32 numVertices = 0;
    physx::PxU32 numTriangles = 0;

    // Pinned host memory buffers for GPU<->CPU transfer
    physx::PxVec4* simPositionInvMassPinned = nullptr;
    physx::PxVec4* simVelocityPinned = nullptr;
    physx::PxVec4* collPositionInvMassPinned = nullptr;
    physx::PxVec4* restPositionPinned = nullptr;

    // Visual properties
    float colorR = 0.2f;
    float colorG = 0.6f;
    float colorB = 1.0f;
};

// Definition for creating soft bodies
struct DeformableVolumeDef
{
    enum class ShapeType { CUBE, SPHERE };

    ShapeType shapeType = ShapeType::CUBE;

    // Transform
    float posX = 0, posY = 5, posZ = 0;
    float scale = 1.0f;

    // Resolution (higher = more detail, slower)
    physx::PxU32 resolution = 8;

    // Material properties
    float youngsModulus = 1e4f;      // Stiffness
    float poissonsRatio = 0.45f;     // Compressibility (0-0.5)
    float dynamicFriction = 0.5f;

    // Damping
    float damping = 0.01f;

    // Color
    float colorR = 0.2f;
    float colorG = 0.6f;
    float colorB = 1.0f;
};

class DeformableVolumeManager
{
public:
    DeformableVolumeManager();
    ~DeformableVolumeManager();

    // Initialize with physics and CUDA context
    bool Initialize(physx::PxPhysics* physics, physx::PxCudaContextManager* cudaContext);

    // Create a soft body from definition
    physx::PxDeformableVolume* CreateVolume(const DeformableVolumeDef& def, physx::PxScene* scene);

    // Update render meshes from GPU data (call each frame after simulation)
    void UpdateRenderMeshes();

    // Get managed volumes for rendering
    const std::vector<ManagedDeformableVolume>& GetVolumes() const { return m_volumes; }

    // Check if soft body simulation is available
    bool IsAvailable() const { return m_isInitialized && m_cudaContext != nullptr; }

    // Release all volumes
    void ReleaseAll(physx::PxScene* scene);

    // Release resources
    void Release();

private:
    physx::PxPhysics* m_physics = nullptr;
    physx::PxCudaContextManager* m_cudaContext = nullptr;

    std::vector<ManagedDeformableVolume> m_volumes;
    std::vector<physx::PxDeformableVolumeMaterial*> m_materials;

    bool m_isInitialized = false;

    // Helper to create tetrahedral mesh for cube shape
    physx::PxDeformableVolumeMesh* CreateCubeMesh(float size, physx::PxU32 resolution);

    // Helper to create tetrahedral mesh for sphere shape
    physx::PxDeformableVolumeMesh* CreateSphereMesh(float radius, physx::PxU32 resolution);
};
