#include "DeformableVolumeManager.h"
#include "extensions/PxDeformableVolumeExt.h"
#include "extensions/PxRemeshingExt.h"
#include "extensions/PxCudaHelpersExt.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include <iostream>
#include <cmath>

using namespace physx;

DeformableVolumeManager::DeformableVolumeManager()
{
}

DeformableVolumeManager::~DeformableVolumeManager()
{
    Release();
}

bool DeformableVolumeManager::Initialize(PxPhysics* physics, PxCudaContextManager* cudaContext)
{
    if (!physics)
    {
        std::cerr << "DeformableVolumeManager: Physics is null" << std::endl;
        return false;
    }

    m_physics = physics;
    m_cudaContext = cudaContext;

    if (!cudaContext)
    {
        std::cerr << "DeformableVolumeManager: CUDA Context Manager is null - soft bodies require GPU" << std::endl;
        return false;
    }

    if (!cudaContext->contextIsValid())
    {
        std::cerr << "DeformableVolumeManager: CUDA context is invalid" << std::endl;
        return false;
    }

    m_isInitialized = true;
    std::cout << "DeformableVolumeManager: Initialized with CUDA support" << std::endl;
    return true;
}

PxDeformableVolume* DeformableVolumeManager::CreateVolume(const DeformableVolumeDef& def, PxScene* scene)
{
    if (!m_isInitialized || !m_cudaContext)
    {
        std::cerr << "DeformableVolumeManager: Not initialized or no CUDA support" << std::endl;
        return nullptr;
    }

    // Create deformable volume mesh
    PxDeformableVolumeMesh* volumeMesh = nullptr;

    if (def.shapeType == DeformableVolumeDef::ShapeType::CUBE)
    {
        volumeMesh = CreateCubeMesh(def.scale, def.resolution);
    }
    else if (def.shapeType == DeformableVolumeDef::ShapeType::SPHERE)
    {
        volumeMesh = CreateSphereMesh(def.scale, def.resolution);
    }

    if (!volumeMesh)
    {
        std::cerr << "DeformableVolumeManager: Failed to create volume mesh" << std::endl;
        return nullptr;
    }

    // Create deformable volume actor
    PxDeformableVolume* volume = m_physics->createDeformableVolume(*m_cudaContext);
    if (!volume)
    {
        std::cerr << "DeformableVolumeManager: Failed to create deformable volume" << std::endl;
        volumeMesh->release();
        return nullptr;
    }

    // Create material
    PxDeformableVolumeMaterial* material = m_physics->createDeformableVolumeMaterial(
        def.youngsModulus,
        def.poissonsRatio,
        def.dynamicFriction
    );

    if (!material)
    {
        std::cerr << "DeformableVolumeManager: Failed to create material" << std::endl;
        volume->release();
        volumeMesh->release();
        return nullptr;
    }
    m_materials.push_back(material);

    // Create shape with tetrahedron mesh geometry
    PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;
    PxTetrahedronMeshGeometry geometry(volumeMesh->getCollisionMesh());
    PxShape* shape = m_physics->createShape(geometry, &material, 1, true, shapeFlags);

    if (!shape)
    {
        std::cerr << "DeformableVolumeManager: Failed to create shape" << std::endl;
        volume->release();
        volumeMesh->release();
        return nullptr;
    }

    // Attach shape to volume
    volume->attachShape(*shape);
    shape->setSimulationFilterData(PxFilterData(0, 0, 2, 0));

    // Attach simulation mesh and aux data
    volume->attachSimulationMesh(*volumeMesh->getSimulationMesh(), *volumeMesh->getDeformableVolumeAuxData());

    // Add to scene before allocating host mirror
    scene->addActor(*volume);

    // Allocate and initialize host mirror buffers
    PxVec4* simPositionInvMassPinned = nullptr;
    PxVec4* simVelocityPinned = nullptr;
    PxVec4* collPositionInvMassPinned = nullptr;
    PxVec4* restPositionPinned = nullptr;

    PxDeformableVolumeExt::allocateAndInitializeHostMirror(
        *volume,
        m_cudaContext,
        simPositionInvMassPinned,
        simVelocityPinned,
        collPositionInvMassPinned,
        restPositionPinned
    );

    // Apply transform
    PxTransform pose(PxVec3(def.posX, def.posY, def.posZ));
    PxReal density = 100.0f;
    PxReal maxInvMassRatio = 50.0f;

    PxDeformableVolumeExt::transform(
        *volume,
        pose,
        1.0f,  // scale
        simPositionInvMassPinned,
        simVelocityPinned,
        collPositionInvMassPinned,
        restPositionPinned
    );

    PxDeformableVolumeExt::updateMass(*volume, density, maxInvMassRatio, simPositionInvMassPinned);

    // Copy to device
    PxDeformableVolumeExt::copyToDevice(
        *volume,
        PxDeformableVolumeDataFlag::eALL,
        simPositionInvMassPinned,
        simVelocityPinned,
        collPositionInvMassPinned,
        restPositionPinned
    );

    // Configure deformable body flags
    volume->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, true);
    volume->setSolverIterationCounts(30);

    // Create managed volume for rendering
    ManagedDeformableVolume managed;
    managed.volume = volume;
    managed.mesh = volumeMesh;
    managed.colorR = def.colorR;
    managed.colorG = def.colorG;
    managed.colorB = def.colorB;

    // Store pinned buffers for later use (for rendering updates)
    managed.simPositionInvMassPinned = simPositionInvMassPinned;
    managed.simVelocityPinned = simVelocityPinned;
    managed.collPositionInvMassPinned = collPositionInvMassPinned;
    managed.restPositionPinned = restPositionPinned;

    // Get collision mesh data for rendering (surface mesh)
    const PxTetrahedronMesh* collisionMesh = volumeMesh->getCollisionMesh();
    if (collisionMesh)
    {
        managed.numVertices = collisionMesh->getNbVertices();

        // Get surface triangles from the collision mesh
        // The collision mesh is a tetrahedron mesh - we need to extract surface triangles
        // For rendering, we'll use the collision mesh vertices which are updated by the simulation
    }

    // Initialize render position buffer
    managed.renderPositions.resize(managed.numVertices);

    m_volumes.push_back(managed);

    std::cout << "DeformableVolumeManager: Created soft body with " << managed.numVertices << " vertices" << std::endl;

    return volume;
}

void DeformableVolumeManager::UpdateRenderMeshes()
{
    if (!m_isInitialized || !m_cudaContext) return;

    for (auto& managed : m_volumes)
    {
        if (!managed.volume || !managed.collPositionInvMassPinned) continue;
        if (managed.numVertices == 0) continue;

        // Get the collision mesh to know the vertex count
        PxTetrahedronMesh* tetMesh = managed.volume->getCollisionMesh();
        if (!tetMesh) continue;

        // Copy deformed vertices from GPU to pinned host memory
        {
            PxScopedCudaLock lock(*m_cudaContext);
            PxVec4* gpuBuffer = managed.volume->getPositionInvMassBufferD();
            if (gpuBuffer)
            {
                m_cudaContext->getCudaContext()->memcpyDtoH(
                    managed.collPositionInvMassPinned,
                    reinterpret_cast<CUdeviceptr>(gpuBuffer),
                    tetMesh->getNbVertices() * sizeof(PxVec4)
                );
            }
        }

        // Copy the collision mesh positions to our render buffer
        for (PxU32 i = 0; i < managed.numVertices; i++)
        {
            managed.renderPositions[i] = managed.collPositionInvMassPinned[i];
        }
    }
}

void DeformableVolumeManager::ReleaseAll(PxScene* scene)
{
    for (auto& managed : m_volumes)
    {
        // Free pinned memory buffers using PhysX extension helper
        if (m_cudaContext)
        {
            if (managed.simPositionInvMassPinned)
                PX_EXT_PINNED_MEMORY_FREE(*m_cudaContext, managed.simPositionInvMassPinned);
            if (managed.simVelocityPinned)
                PX_EXT_PINNED_MEMORY_FREE(*m_cudaContext, managed.simVelocityPinned);
            if (managed.collPositionInvMassPinned)
                PX_EXT_PINNED_MEMORY_FREE(*m_cudaContext, managed.collPositionInvMassPinned);
            if (managed.restPositionPinned)
                PX_EXT_PINNED_MEMORY_FREE(*m_cudaContext, managed.restPositionPinned);
        }

        if (managed.volume && scene)
        {
            scene->removeActor(*managed.volume);
            managed.volume->release();
        }
        if (managed.mesh)
        {
            managed.mesh->release();
        }
    }
    m_volumes.clear();

    for (auto* material : m_materials)
    {
        if (material)
        {
            material->release();
        }
    }
    m_materials.clear();
}

void DeformableVolumeManager::Release()
{
    // Note: Volumes should be released from scene first via ReleaseAll
    m_volumes.clear();
    m_materials.clear();
    m_isInitialized = false;
}

PxDeformableVolumeMesh* DeformableVolumeManager::CreateCubeMesh(float size, PxU32 resolution)
{
    // Create a simple cube surface mesh, then let PhysX tetrahedralize it
    const float halfSize = size * 0.5f;

    // Cube vertices
    std::vector<PxVec3> vertices = {
        // Front face
        PxVec3(-halfSize, -halfSize,  halfSize),
        PxVec3( halfSize, -halfSize,  halfSize),
        PxVec3( halfSize,  halfSize,  halfSize),
        PxVec3(-halfSize,  halfSize,  halfSize),
        // Back face
        PxVec3(-halfSize, -halfSize, -halfSize),
        PxVec3( halfSize, -halfSize, -halfSize),
        PxVec3( halfSize,  halfSize, -halfSize),
        PxVec3(-halfSize,  halfSize, -halfSize)
    };

    // Cube triangles (12 triangles for 6 faces)
    std::vector<PxU32> indices = {
        // Front
        0, 1, 2, 0, 2, 3,
        // Back
        5, 4, 7, 5, 7, 6,
        // Left
        4, 0, 3, 4, 3, 7,
        // Right
        1, 5, 6, 1, 6, 2,
        // Top
        3, 2, 6, 3, 6, 7,
        // Bottom
        4, 5, 1, 4, 1, 0
    };

    // Create simple mesh descriptor
    PxSimpleTriangleMesh surfaceMesh;
    surfaceMesh.points.count = static_cast<PxU32>(vertices.size());
    surfaceMesh.points.data = vertices.data();
    surfaceMesh.points.stride = sizeof(PxVec3);
    surfaceMesh.triangles.count = static_cast<PxU32>(indices.size() / 3);
    surfaceMesh.triangles.data = indices.data();
    surfaceMesh.triangles.stride = 3 * sizeof(PxU32);

    // Cooking parameters
    PxCookingParams cookingParams(m_physics->getTolerancesScale());
    cookingParams.meshWeldTolerance = 0.001f;
    cookingParams.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
    cookingParams.buildGPUData = true;

    // Create deformable volume mesh using extension
    PxDeformableVolumeMesh* volumeMesh = PxDeformableVolumeExt::createDeformableVolumeMesh(
        cookingParams,
        surfaceMesh,
        resolution,
        m_physics->getPhysicsInsertionCallback()
    );

    return volumeMesh;
}

PxDeformableVolumeMesh* DeformableVolumeManager::CreateSphereMesh(float radius, PxU32 resolution)
{
    // Create sphere surface mesh using lat-long parameterization
    const int latSegments = 16;
    const int lonSegments = 16;

    std::vector<PxVec3> vertices;
    std::vector<PxU32> indices;

    // Generate vertices
    for (int lat = 0; lat <= latSegments; lat++)
    {
        float theta = lat * PxPi / latSegments;
        float sinTheta = sinf(theta);
        float cosTheta = cosf(theta);

        for (int lon = 0; lon <= lonSegments; lon++)
        {
            float phi = lon * 2.0f * PxPi / lonSegments;
            float sinPhi = sinf(phi);
            float cosPhi = cosf(phi);

            float x = cosPhi * sinTheta;
            float y = cosTheta;
            float z = sinPhi * sinTheta;

            vertices.push_back(PxVec3(x * radius, y * radius, z * radius));
        }
    }

    // Generate indices
    for (int lat = 0; lat < latSegments; lat++)
    {
        for (int lon = 0; lon < lonSegments; lon++)
        {
            int current = lat * (lonSegments + 1) + lon;
            int next = current + lonSegments + 1;

            indices.push_back(current);
            indices.push_back(next);
            indices.push_back(current + 1);

            indices.push_back(current + 1);
            indices.push_back(next);
            indices.push_back(next + 1);
        }
    }

    // Create simple mesh descriptor
    PxSimpleTriangleMesh surfaceMesh;
    surfaceMesh.points.count = static_cast<PxU32>(vertices.size());
    surfaceMesh.points.data = vertices.data();
    surfaceMesh.points.stride = sizeof(PxVec3);
    surfaceMesh.triangles.count = static_cast<PxU32>(indices.size() / 3);
    surfaceMesh.triangles.data = indices.data();
    surfaceMesh.triangles.stride = 3 * sizeof(PxU32);

    // Cooking parameters
    PxCookingParams cookingParams(m_physics->getTolerancesScale());
    cookingParams.meshWeldTolerance = 0.001f;
    cookingParams.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
    cookingParams.buildGPUData = true;

    // Create deformable volume mesh
    PxDeformableVolumeMesh* volumeMesh = PxDeformableVolumeExt::createDeformableVolumeMesh(
        cookingParams,
        surfaceMesh,
        resolution,
        m_physics->getPhysicsInsertionCallback()
    );

    return volumeMesh;
}
