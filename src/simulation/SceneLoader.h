#pragma once

#include <PxPhysicsAPI.h>
#include <string>
#include <vector>
#include <memory>

// Note: Avoid 'using namespace' in headers - use explicit physx:: prefix instead

// Scene definition structures
struct ActorDef
{
    enum class Type { STATIC, DYNAMIC };
    enum class GeomType { BOX, SPHERE, PLANE, CAPSULE, CONVEX_MESH, TRIANGLE_MESH, HEIGHTFIELD };

    Type type;
    GeomType geomType;

    // Transform
    float posX, posY, posZ;
    float quatW, quatX, quatY, quatZ;

    // Geometry parameters
    float boxHalfX, boxHalfY, boxHalfZ;  // For box
    float sphereRadius;                   // For sphere

    // Capsule parameters
    float capsuleRadius;
    float capsuleHalfHeight;

    // Mesh parameters
    std::string meshFile;  // Path to OBJ file for convex/triangle mesh

    // Heightfield parameters
    int heightfieldRows;
    int heightfieldCols;
    float heightfieldRowScale;
    float heightfieldColScale;
    float heightfieldHeightScale;

    // Physics properties
    float density;
    float staticFriction;
    float dynamicFriction;
    float restitution;

    // Initial velocity (for dynamic actors)
    float velX, velY, velZ;
    float angVelX, angVelY, angVelZ;

    // Name for identification
    std::string name;
};

struct JointDef
{
    enum class Type { FIXED, SPHERICAL, REVOLUTE, PRISMATIC, DISTANCE, D6 };

    Type type;
    std::string actor1Name;  // Reference to actor by name
    std::string actor2Name;  // Can be empty for world attachment

    // Local frames for joint attachment
    float frame1PosX, frame1PosY, frame1PosZ;
    float frame1QuatW, frame1QuatX, frame1QuatY, frame1QuatZ;
    float frame2PosX, frame2PosY, frame2PosZ;
    float frame2QuatW, frame2QuatX, frame2QuatY, frame2QuatZ;

    // Joint-specific parameters
    bool limitEnabled;
    float limitLower, limitUpper;  // For revolute/prismatic
    float limitConeAngle;          // For spherical (cone limit)

    bool driveEnabled;
    float driveStiffness;
    float driveDamping;
    float driveTargetValue;

    bool breakable;
    float breakForce;
    float breakTorque;

    // Distance joint parameters
    float minDistance;
    float maxDistance;
    float springStiffness;
    float springDamping;

    std::string name;
};

// Articulation link definition for robot arms and ragdolls
struct ArticulationLinkDef
{
    std::string name;
    std::string parentName;

    // Link shape (uses ActorDef geometry params)
    ActorDef::GeomType geomType;
    float boxHalfX, boxHalfY, boxHalfZ;
    float sphereRadius;
    float capsuleRadius, capsuleHalfHeight;

    // Joint type and frame
    physx::PxArticulationJointType::Enum jointType;
    float localPosX, localPosY, localPosZ;
    float parentPosX, parentPosY, parentPosZ;

    // Joint limits (for each axis)
    float twistLimitLow, twistLimitHigh;
    float swing1Limit, swing2Limit;

    // Drive parameters
    bool driveEnabled;
    float driveStiffness;
    float driveDamping;
    float driveTarget;

    // Physics properties
    float density;
};

struct ArticulationDef
{
    std::string name;
    bool fixBase;
    std::vector<ArticulationLinkDef> links;
};

struct SceneDefinition
{
    std::string name;
    std::string description;

    // Global scene properties
    float gravityX, gravityY, gravityZ;

    // Actors and joints
    std::vector<ActorDef> actors;
    std::vector<JointDef> joints;
    std::vector<ArticulationDef> articulations;
};

class SceneLoader
{
public:
    SceneLoader();
    ~SceneLoader();

    // Load scene definition from JSON file
    bool LoadFromJSON(const std::string& filename, SceneDefinition& sceneDef);

    // Create PhysX scene from definition
    bool CreateScene(physx::PxPhysics* physics, const SceneDefinition& sceneDef,
                     physx::PxScene* scene, physx::PxMaterial* defaultMaterial,
                     std::vector<physx::PxRigidActor*>& outActors,
                     std::vector<physx::PxJoint*>& outJoints);

    // Save current scene to JSON
    bool SaveToJSON(const std::string& filename, physx::PxScene* scene);

    // Get last error message
    const std::string& GetLastError() const { return m_lastError; }

private:
    std::string m_lastError;

    // Helper functions for parsing
    bool ParseActorDef(const std::string& jsonStr, size_t& pos, ActorDef& actor);
    bool ParseJointDef(const std::string& jsonStr, size_t& pos, JointDef& joint);

    // Helper functions for creation
    physx::PxRigidActor* CreateActor(physx::PxPhysics* physics, const ActorDef& def, physx::PxMaterial* material);
    physx::PxJoint* CreateJoint(physx::PxPhysics* physics, const JointDef& def,
                        physx::PxRigidActor* actor1, physx::PxRigidActor* actor2);

    // Simple JSON parsing helpers (basic implementation)
    // TODO: Consider replacing with a robust JSON library such as:
    //   - nlohmann/json (header-only, modern C++, widely used)
    //   - RapidJSON (high performance, SAX/DOM parsing)
    //   - simdjson (SIMD-optimized, extremely fast for large files)
    // Current implementation lacks: nested object support, array handling,
    // escape sequence processing, and comprehensive error reporting.
    bool FindKey(const std::string& json, size_t& pos, const std::string& key);
    bool ReadString(const std::string& json, size_t& pos, std::string& value);
    bool ReadFloat(const std::string& json, size_t& pos, float& value);
    bool ReadInt(const std::string& json, size_t& pos, int& value);
    bool ReadBool(const std::string& json, size_t& pos, bool& value);
    void SkipWhitespace(const std::string& json, size_t& pos);
};
