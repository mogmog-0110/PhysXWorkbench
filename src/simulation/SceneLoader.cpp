#include "SceneLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>

using namespace physx;

SceneLoader::SceneLoader()
{
}

SceneLoader::~SceneLoader()
{
}

bool SceneLoader::LoadFromJSON(const std::string& filename, SceneDefinition& sceneDef)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        m_lastError = "Failed to open file: " + filename;
        return false;
    }

    // Read entire file
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonContent = buffer.str();
    file.close();

    // Simple JSON parsing - look for key patterns
    // This is a basic implementation; for production, use a proper JSON library like nlohmann/json

    size_t pos = 0;

    // Parse scene name
    if (FindKey(jsonContent, pos, "name"))
    {
        ReadString(jsonContent, pos, sceneDef.name);
    }

    // Parse description
    pos = 0;
    if (FindKey(jsonContent, pos, "description"))
    {
        ReadString(jsonContent, pos, sceneDef.description);
    }

    // Parse gravity
    pos = 0;
    if (FindKey(jsonContent, pos, "gravity"))
    {
        SkipWhitespace(jsonContent, pos);
        if (pos < jsonContent.length() && jsonContent[pos] == '[')
        {
            pos++;
            ReadFloat(jsonContent, pos, sceneDef.gravityX);
            ReadFloat(jsonContent, pos, sceneDef.gravityY);
            ReadFloat(jsonContent, pos, sceneDef.gravityZ);
        }
    }

    // Parse actors array
    pos = 0;
    if (FindKey(jsonContent, pos, "actors"))
    {
        SkipWhitespace(jsonContent, pos);
        if (pos < jsonContent.length() && jsonContent[pos] == '[')
        {
            pos++;
            while (pos < jsonContent.length())
            {
                SkipWhitespace(jsonContent, pos);
                if (pos >= jsonContent.length()) break;
                if (jsonContent[pos] == ']') break;
                if (jsonContent[pos] == '{')
                {
                    ActorDef actor = {};
                    if (ParseActorDef(jsonContent, pos, actor))
                    {
                        sceneDef.actors.push_back(actor);
                    }
                }
                SkipWhitespace(jsonContent, pos);
                if (pos < jsonContent.length() && jsonContent[pos] == ',') pos++;
            }
        }
    }

    // Parse joints array
    pos = 0;
    if (FindKey(jsonContent, pos, "joints"))
    {
        SkipWhitespace(jsonContent, pos);
        if (pos < jsonContent.length() && jsonContent[pos] == '[')
        {
            pos++;
            while (pos < jsonContent.length())
            {
                SkipWhitespace(jsonContent, pos);
                if (pos >= jsonContent.length()) break;
                if (jsonContent[pos] == ']') break;
                if (jsonContent[pos] == '{')
                {
                    JointDef joint = {};
                    if (ParseJointDef(jsonContent, pos, joint))
                    {
                        sceneDef.joints.push_back(joint);
                    }
                }
                SkipWhitespace(jsonContent, pos);
                if (pos < jsonContent.length() && jsonContent[pos] == ',') pos++;
            }
        }
    }

    std::cout << "Loaded scene: " << sceneDef.name << std::endl;
    std::cout << "  Actors: " << sceneDef.actors.size() << std::endl;
    std::cout << "  Joints: " << sceneDef.joints.size() << std::endl;

    return true;
}

bool SceneLoader::ParseActorDef(const std::string& json, size_t& pos, ActorDef& actor)
{
    // Default values
    actor.type = ActorDef::Type::DYNAMIC;
    actor.geomType = ActorDef::GeomType::BOX;
    actor.posX = actor.posY = actor.posZ = 0.0f;
    actor.quatW = 1.0f; actor.quatX = actor.quatY = actor.quatZ = 0.0f;
    actor.boxHalfX = actor.boxHalfY = actor.boxHalfZ = 0.5f;
    actor.sphereRadius = 0.5f;
    actor.capsuleRadius = 0.5f;
    actor.capsuleHalfHeight = 1.0f;
    actor.heightfieldRows = 16;
    actor.heightfieldCols = 16;
    actor.heightfieldRowScale = 1.0f;
    actor.heightfieldColScale = 1.0f;
    actor.heightfieldHeightScale = 1.0f;
    actor.density = 1.0f;
    actor.staticFriction = 0.5f;
    actor.dynamicFriction = 0.5f;
    actor.restitution = 0.3f;
    actor.velX = actor.velY = actor.velZ = 0.0f;
    actor.angVelX = actor.angVelY = actor.angVelZ = 0.0f;

    size_t start = pos;
    size_t objEnd = json.find('}', start);
    if (objEnd == std::string::npos) return false;

    std::string objStr = json.substr(start, objEnd - start + 1);
    size_t localPos = 0;

    // Parse type
    if (FindKey(objStr, localPos, "type"))
    {
        std::string typeStr;
        ReadString(objStr, localPos, typeStr);
        if (typeStr == "static") actor.type = ActorDef::Type::STATIC;
        else if (typeStr == "dynamic") actor.type = ActorDef::Type::DYNAMIC;
    }

    // Parse geometry type
    localPos = 0;
    if (FindKey(objStr, localPos, "geometry"))
    {
        std::string geomStr;
        ReadString(objStr, localPos, geomStr);
        if (geomStr == "box") actor.geomType = ActorDef::GeomType::BOX;
        else if (geomStr == "sphere") actor.geomType = ActorDef::GeomType::SPHERE;
        else if (geomStr == "plane") actor.geomType = ActorDef::GeomType::PLANE;
        else if (geomStr == "capsule") actor.geomType = ActorDef::GeomType::CAPSULE;
        else if (geomStr == "convex_mesh") actor.geomType = ActorDef::GeomType::CONVEX_MESH;
        else if (geomStr == "triangle_mesh") actor.geomType = ActorDef::GeomType::TRIANGLE_MESH;
        else if (geomStr == "heightfield") actor.geomType = ActorDef::GeomType::HEIGHTFIELD;
    }

    // Parse name
    localPos = 0;
    if (FindKey(objStr, localPos, "name"))
    {
        ReadString(objStr, localPos, actor.name);
    }

    // Parse position
    localPos = 0;
    if (FindKey(objStr, localPos, "position"))
    {
        SkipWhitespace(objStr, localPos);
        if (objStr[localPos] == '[')
        {
            localPos++;
            ReadFloat(objStr, localPos, actor.posX);
            ReadFloat(objStr, localPos, actor.posY);
            ReadFloat(objStr, localPos, actor.posZ);
        }
    }

    // Parse box dimensions
    if (actor.geomType == ActorDef::GeomType::BOX)
    {
        localPos = 0;
        if (FindKey(objStr, localPos, "halfExtents"))
        {
            SkipWhitespace(objStr, localPos);
            if (objStr[localPos] == '[')
            {
                localPos++;
                ReadFloat(objStr, localPos, actor.boxHalfX);
                ReadFloat(objStr, localPos, actor.boxHalfY);
                ReadFloat(objStr, localPos, actor.boxHalfZ);
            }
        }
    }

    // Parse sphere radius
    if (actor.geomType == ActorDef::GeomType::SPHERE)
    {
        localPos = 0;
        if (FindKey(objStr, localPos, "radius"))
        {
            ReadFloat(objStr, localPos, actor.sphereRadius);
        }
    }

    // Parse capsule parameters
    if (actor.geomType == ActorDef::GeomType::CAPSULE)
    {
        localPos = 0;
        if (FindKey(objStr, localPos, "capsuleRadius"))
        {
            ReadFloat(objStr, localPos, actor.capsuleRadius);
        }
        else if (FindKey(objStr, localPos, "radius"))
        {
            ReadFloat(objStr, localPos, actor.capsuleRadius);
        }

        localPos = 0;
        if (FindKey(objStr, localPos, "capsuleHalfHeight"))
        {
            ReadFloat(objStr, localPos, actor.capsuleHalfHeight);
        }
        else if (FindKey(objStr, localPos, "halfHeight"))
        {
            ReadFloat(objStr, localPos, actor.capsuleHalfHeight);
        }
    }

    // Parse mesh file path
    if (actor.geomType == ActorDef::GeomType::CONVEX_MESH ||
        actor.geomType == ActorDef::GeomType::TRIANGLE_MESH)
    {
        localPos = 0;
        if (FindKey(objStr, localPos, "meshFile"))
        {
            ReadString(objStr, localPos, actor.meshFile);
        }
    }

    // Parse heightfield parameters
    if (actor.geomType == ActorDef::GeomType::HEIGHTFIELD)
    {
        localPos = 0;
        if (FindKey(objStr, localPos, "rows"))
        {
            float fval;
            ReadFloat(objStr, localPos, fval);
            actor.heightfieldRows = static_cast<int>(fval);
        }

        localPos = 0;
        if (FindKey(objStr, localPos, "cols"))
        {
            float fval;
            ReadFloat(objStr, localPos, fval);
            actor.heightfieldCols = static_cast<int>(fval);
        }

        localPos = 0;
        if (FindKey(objStr, localPos, "rowScale"))
        {
            ReadFloat(objStr, localPos, actor.heightfieldRowScale);
        }

        localPos = 0;
        if (FindKey(objStr, localPos, "colScale"))
        {
            ReadFloat(objStr, localPos, actor.heightfieldColScale);
        }

        localPos = 0;
        if (FindKey(objStr, localPos, "heightScale"))
        {
            ReadFloat(objStr, localPos, actor.heightfieldHeightScale);
        }
    }

    // Parse physics properties
    localPos = 0;
    if (FindKey(objStr, localPos, "density"))
    {
        ReadFloat(objStr, localPos, actor.density);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "friction"))
    {
        ReadFloat(objStr, localPos, actor.staticFriction);
        actor.dynamicFriction = actor.staticFriction;
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "restitution"))
    {
        ReadFloat(objStr, localPos, actor.restitution);
    }

    pos = objEnd + 1;
    return true;
}

bool SceneLoader::ParseJointDef(const std::string& json, size_t& pos, JointDef& joint)
{
    // Default values
    joint.type = JointDef::Type::FIXED;
    joint.frame1PosX = joint.frame1PosY = joint.frame1PosZ = 0.0f;
    joint.frame1QuatW = 1.0f; joint.frame1QuatX = joint.frame1QuatY = joint.frame1QuatZ = 0.0f;
    joint.frame2PosX = joint.frame2PosY = joint.frame2PosZ = 0.0f;
    joint.frame2QuatW = 1.0f; joint.frame2QuatX = joint.frame2QuatY = joint.frame2QuatZ = 0.0f;
    joint.limitEnabled = false;
    joint.limitLower = 0.0f;
    joint.limitUpper = 0.0f;
    joint.limitConeAngle = 0.785398f;  // 45 degrees default
    joint.driveEnabled = false;
    joint.driveStiffness = 0.0f;
    joint.driveDamping = 0.0f;
    joint.driveTargetValue = 0.0f;
    joint.breakable = false;
    joint.breakForce = FLT_MAX;
    joint.breakTorque = FLT_MAX;

    // Distance joint defaults
    joint.minDistance = 0.0f;
    joint.maxDistance = 0.0f;
    joint.springStiffness = 0.0f;
    joint.springDamping = 0.0f;

    size_t start = pos;
    size_t objEnd = json.find('}', start);
    if (objEnd == std::string::npos) return false;

    std::string objStr = json.substr(start, objEnd - start + 1);
    size_t localPos = 0;

    // Parse type
    if (FindKey(objStr, localPos, "type"))
    {
        std::string typeStr;
        ReadString(objStr, localPos, typeStr);
        if (typeStr == "fixed") joint.type = JointDef::Type::FIXED;
        else if (typeStr == "spherical") joint.type = JointDef::Type::SPHERICAL;
        else if (typeStr == "revolute") joint.type = JointDef::Type::REVOLUTE;
        else if (typeStr == "prismatic") joint.type = JointDef::Type::PRISMATIC;
        else if (typeStr == "distance") joint.type = JointDef::Type::DISTANCE;
        else if (typeStr == "d6") joint.type = JointDef::Type::D6;
    }

    // Parse actor names
    localPos = 0;
    if (FindKey(objStr, localPos, "actor1"))
    {
        ReadString(objStr, localPos, joint.actor1Name);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "actor2"))
    {
        ReadString(objStr, localPos, joint.actor2Name);
    }

    // Parse name
    localPos = 0;
    if (FindKey(objStr, localPos, "name"))
    {
        ReadString(objStr, localPos, joint.name);
    }

    // Parse breakable
    localPos = 0;
    if (FindKey(objStr, localPos, "breakable"))
    {
        ReadBool(objStr, localPos, joint.breakable);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "breakForce"))
    {
        ReadFloat(objStr, localPos, joint.breakForce);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "breakTorque"))
    {
        ReadFloat(objStr, localPos, joint.breakTorque);
    }

    // Parse limit parameters
    localPos = 0;
    if (FindKey(objStr, localPos, "limitEnabled"))
    {
        ReadBool(objStr, localPos, joint.limitEnabled);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "limitLower"))
    {
        ReadFloat(objStr, localPos, joint.limitLower);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "limitUpper"))
    {
        ReadFloat(objStr, localPos, joint.limitUpper);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "limitConeAngle"))
    {
        ReadFloat(objStr, localPos, joint.limitConeAngle);
    }

    // Parse drive parameters
    localPos = 0;
    if (FindKey(objStr, localPos, "driveEnabled"))
    {
        ReadBool(objStr, localPos, joint.driveEnabled);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "driveStiffness"))
    {
        ReadFloat(objStr, localPos, joint.driveStiffness);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "driveDamping"))
    {
        ReadFloat(objStr, localPos, joint.driveDamping);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "driveTargetValue"))
    {
        ReadFloat(objStr, localPos, joint.driveTargetValue);
    }

    // Parse distance joint parameters
    localPos = 0;
    if (FindKey(objStr, localPos, "minDistance"))
    {
        ReadFloat(objStr, localPos, joint.minDistance);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "maxDistance"))
    {
        ReadFloat(objStr, localPos, joint.maxDistance);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "springStiffness"))
    {
        ReadFloat(objStr, localPos, joint.springStiffness);
    }

    localPos = 0;
    if (FindKey(objStr, localPos, "springDamping"))
    {
        ReadFloat(objStr, localPos, joint.springDamping);
    }

    pos = objEnd + 1;
    return true;
}

bool SceneLoader::CreateScene(PxPhysics* physics, const SceneDefinition& sceneDef,
                              PxScene* scene, PxMaterial* defaultMaterial,
                              std::vector<PxRigidActor*>& outActors,
                              std::vector<PxJoint*>& outJoints)
{
    if (!physics || !scene || !defaultMaterial)
    {
        m_lastError = "Invalid physics, scene, or material";
        return false;
    }

    // Set gravity
    scene->setGravity(PxVec3(sceneDef.gravityX, sceneDef.gravityY, sceneDef.gravityZ));

    // Map actor names to created actors
    std::unordered_map<std::string, PxRigidActor*> actorMap;

    // Create all actors
    for (const auto& actorDef : sceneDef.actors)
    {
        PxRigidActor* actor = CreateActor(physics, actorDef, defaultMaterial);
        if (actor)
        {
            scene->addActor(*actor);
            outActors.push_back(actor);
            if (!actorDef.name.empty())
            {
                actorMap[actorDef.name] = actor;
            }
        }
    }

    // Create all joints
    for (const auto& jointDef : sceneDef.joints)
    {
        PxRigidActor* actor1 = nullptr;
        PxRigidActor* actor2 = nullptr;

        if (!jointDef.actor1Name.empty())
        {
            auto it = actorMap.find(jointDef.actor1Name);
            if (it != actorMap.end()) actor1 = it->second;
        }

        if (!jointDef.actor2Name.empty())
        {
            auto it = actorMap.find(jointDef.actor2Name);
            if (it != actorMap.end()) actor2 = it->second;
        }

        PxJoint* joint = CreateJoint(physics, jointDef, actor1, actor2);
        if (joint)
        {
            outJoints.push_back(joint);
        }
    }

    std::cout << "Created scene with " << outActors.size() << " actors and "
              << outJoints.size() << " joints" << std::endl;

    return true;
}

PxRigidActor* SceneLoader::CreateActor(PxPhysics* physics, const ActorDef& def, PxMaterial* material)
{
    PxTransform transform(PxVec3(def.posX, def.posY, def.posZ),
                          PxQuat(def.quatX, def.quatY, def.quatZ, def.quatW));

    PxRigidActor* actor = nullptr;

    if (def.type == ActorDef::Type::STATIC)
    {
        if (def.geomType == ActorDef::GeomType::PLANE)
        {
            actor = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
        }
        else
        {
            actor = physics->createRigidStatic(transform);
        }
    }
    else
    {
        PxRigidDynamic* dynamicActor = physics->createRigidDynamic(transform);
        if (dynamicActor)
        {
            // Set initial velocities
            dynamicActor->setLinearVelocity(PxVec3(def.velX, def.velY, def.velZ));
            dynamicActor->setAngularVelocity(PxVec3(def.angVelX, def.angVelY, def.angVelZ));
        }
        actor = dynamicActor;
    }

    if (!actor) return nullptr;

    // Create and attach shape (except for plane which already has a shape)
    if (def.geomType != ActorDef::GeomType::PLANE)
    {
        PxShape* shape = nullptr;

        if (def.geomType == ActorDef::GeomType::BOX)
        {
            PxBoxGeometry boxGeom(def.boxHalfX, def.boxHalfY, def.boxHalfZ);
            shape = PxRigidActorExt::createExclusiveShape(*actor, boxGeom, *material);
        }
        else if (def.geomType == ActorDef::GeomType::SPHERE)
        {
            PxSphereGeometry sphereGeom(def.sphereRadius);
            shape = PxRigidActorExt::createExclusiveShape(*actor, sphereGeom, *material);
        }
        else if (def.geomType == ActorDef::GeomType::CAPSULE)
        {
            // PhysX capsule is aligned along X axis by default
            // Radius is for the hemisphere caps, halfHeight is the half-length of the cylinder part
            PxCapsuleGeometry capsuleGeom(def.capsuleRadius, def.capsuleHalfHeight);
            shape = PxRigidActorExt::createExclusiveShape(*actor, capsuleGeom, *material);
        }
        else if (def.geomType == ActorDef::GeomType::CONVEX_MESH)
        {
            // Create a simple procedural convex mesh (pyramid) for demonstration
            // In production, this would load from meshFile
            float scale = def.boxHalfX > 0 ? def.boxHalfX : 1.0f;  // Use boxHalfX as scale
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
            if (PxCookConvexMesh(PxCookingParams(physics->getTolerancesScale()), convexDesc, buf, &result))
            {
                PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
                PxConvexMesh* convexMesh = physics->createConvexMesh(input);
                if (convexMesh)
                {
                    PxConvexMeshGeometry convexGeom(convexMesh);
                    shape = PxRigidActorExt::createExclusiveShape(*actor, convexGeom, *material);
                }
            }
        }
        else if (def.geomType == ActorDef::GeomType::TRIANGLE_MESH)
        {
            // Triangle mesh can only be used with static actors
            if (def.type == ActorDef::Type::STATIC)
            {
                // Create a simple triangle mesh (ground ramp) for demonstration
                float scale = def.boxHalfX > 0 ? def.boxHalfX : 5.0f;
                PxVec3 rampVerts[] = {
                    PxVec3(-scale, 0, -scale),
                    PxVec3(scale, 0, -scale),
                    PxVec3(scale, scale * 0.5f, scale),
                    PxVec3(-scale, scale * 0.5f, scale)
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
                if (PxCookTriangleMesh(PxCookingParams(physics->getTolerancesScale()), meshDesc, buf, &result))
                {
                    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
                    PxTriangleMesh* triMesh = physics->createTriangleMesh(input);
                    if (triMesh)
                    {
                        PxTriangleMeshGeometry triGeom(triMesh);
                        shape = PxRigidActorExt::createExclusiveShape(*actor, triGeom, *material);
                    }
                }
            }
        }
        else if (def.geomType == ActorDef::GeomType::HEIGHTFIELD)
        {
            // Heightfield can only be used with static actors
            if (def.type == ActorDef::Type::STATIC)
            {
                int rows = def.heightfieldRows > 0 ? def.heightfieldRows : 32;
                int cols = def.heightfieldCols > 0 ? def.heightfieldCols : 32;
                float rowScale = def.heightfieldRowScale > 0 ? def.heightfieldRowScale : 1.0f;
                float colScale = def.heightfieldColScale > 0 ? def.heightfieldColScale : 1.0f;
                float heightScale = def.heightfieldHeightScale > 0 ? def.heightfieldHeightScale : 1.0f;

                // Generate procedural terrain heights (sine-based hills)
                std::vector<PxHeightFieldSample> samples(rows * cols);
                for (int r = 0; r < rows; r++)
                {
                    for (int c = 0; c < cols; c++)
                    {
                        // Create hills using sine waves
                        float fx = static_cast<float>(c) / cols * 4.0f * 3.14159f;
                        float fz = static_cast<float>(r) / rows * 4.0f * 3.14159f;
                        float h = (sinf(fx) + sinf(fz) + 2.0f) * 0.5f;  // Range [0, 2]

                        PxHeightFieldSample& sample = samples[r * cols + c];
                        sample.height = static_cast<PxI16>(h * 32767.0f / 2.0f);  // Scale to int16 range
                        sample.materialIndex0 = 0;
                        sample.materialIndex1 = 0;
                    }
                }

                PxHeightFieldDesc hfDesc;
                hfDesc.nbRows = rows;
                hfDesc.nbColumns = cols;
                hfDesc.samples.data = samples.data();
                hfDesc.samples.stride = sizeof(PxHeightFieldSample);

                PxHeightField* heightField = PxCreateHeightField(hfDesc, physics->getPhysicsInsertionCallback());
                if (heightField)
                {
                    PxHeightFieldGeometry hfGeom(heightField, PxMeshGeometryFlags(), heightScale, rowScale, colScale);
                    shape = PxRigidActorExt::createExclusiveShape(*actor, hfGeom, *material);
                }
            }
        }

        if (shape)
        {
            shape->setSimulationFilterData(PxFilterData(1, 1, 0, 0));

            // Update mass (for dynamic actors)
            if (def.type == ActorDef::Type::DYNAMIC)
            {
                PxRigidBodyExt::updateMassAndInertia(*actor->is<PxRigidDynamic>(), def.density);
            }
        }
    }

    return actor;
}

PxJoint* SceneLoader::CreateJoint(PxPhysics* physics, const JointDef& def,
                                 PxRigidActor* actor1, PxRigidActor* actor2)
{
    PxTransform frame1(PxVec3(def.frame1PosX, def.frame1PosY, def.frame1PosZ),
                       PxQuat(def.frame1QuatX, def.frame1QuatY, def.frame1QuatZ, def.frame1QuatW));
    PxTransform frame2(PxVec3(def.frame2PosX, def.frame2PosY, def.frame2PosZ),
                       PxQuat(def.frame2QuatX, def.frame2QuatY, def.frame2QuatZ, def.frame2QuatW));

    PxJoint* joint = nullptr;

    switch (def.type)
    {
    case JointDef::Type::FIXED:
        joint = PxFixedJointCreate(*physics, actor1, frame1, actor2, frame2);
        break;

    case JointDef::Type::SPHERICAL:
    {
        PxSphericalJoint* spherical = PxSphericalJointCreate(*physics, actor1, frame1, actor2, frame2);
        if (spherical && def.limitEnabled)
        {
            spherical->setLimitCone(PxJointLimitCone(def.limitConeAngle, def.limitConeAngle));
            spherical->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
        }
        joint = spherical;
        break;
    }

    case JointDef::Type::REVOLUTE:
    {
        PxRevoluteJoint* revolute = PxRevoluteJointCreate(*physics, actor1, frame1, actor2, frame2);
        if (revolute)
        {
            // Configure limits
            if (def.limitEnabled)
            {
                PxJointAngularLimitPair limitPair(def.limitLower, def.limitUpper);
                revolute->setLimit(limitPair);
                revolute->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
            }

            // Configure drive (motor)
            if (def.driveEnabled)
            {
                revolute->setDriveVelocity(def.driveTargetValue);
                revolute->setDriveForceLimit(PX_MAX_F32);
                revolute->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
            }
        }
        joint = revolute;
        break;
    }

    case JointDef::Type::PRISMATIC:
    {
        PxPrismaticJoint* prismatic = PxPrismaticJointCreate(*physics, actor1, frame1, actor2, frame2);
        if (prismatic)
        {
            // Configure limits (PxJointLinearLimitPair requires PxTolerancesScale in PhysX 5.x)
            if (def.limitEnabled)
            {
                PxTolerancesScale tolerances = physics->getTolerancesScale();
                PxJointLinearLimitPair limitPair(tolerances, def.limitLower, def.limitUpper);
                prismatic->setLimit(limitPair);
                prismatic->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
            }
        }
        joint = prismatic;
        break;
    }

    case JointDef::Type::DISTANCE:
    {
        PxDistanceJoint* distance = PxDistanceJointCreate(*physics, actor1, frame1, actor2, frame2);
        if (distance)
        {
            // Set min/max distance
            if (def.minDistance > 0.0f)
            {
                distance->setMinDistance(def.minDistance);
                distance->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
            }
            if (def.maxDistance > 0.0f)
            {
                distance->setMaxDistance(def.maxDistance);
                distance->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
            }

            // Configure spring if stiffness is set
            if (def.springStiffness > 0.0f)
            {
                distance->setStiffness(def.springStiffness);
                distance->setDamping(def.springDamping);
                distance->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
            }
        }
        joint = distance;
        break;
    }

    case JointDef::Type::D6:
    {
        PxD6Joint* d6 = PxD6JointCreate(*physics, actor1, frame1, actor2, frame2);
        if (d6 && def.driveEnabled)
        {
            d6->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(def.driveStiffness, def.driveDamping, FLT_MAX, true));
        }
        joint = d6;
        break;
    }

    default:
        break;
    }

    if (joint && def.breakable)
    {
        joint->setBreakForce(def.breakForce, def.breakTorque);
    }

    return joint;
}

// Simple JSON parsing helper functions
bool SceneLoader::FindKey(const std::string& json, size_t& pos, const std::string& key)
{
    std::string searchStr = "\"" + key + "\"";
    size_t found = json.find(searchStr, pos);
    if (found != std::string::npos)
    {
        pos = found + searchStr.length();
        SkipWhitespace(json, pos);
        if (json[pos] == ':')
        {
            pos++;
            SkipWhitespace(json, pos);
            return true;
        }
    }
    return false;
}

bool SceneLoader::ReadString(const std::string& json, size_t& pos, std::string& value)
{
    SkipWhitespace(json, pos);
    if (pos >= json.length() || json[pos] != '\"') return false;

    pos++;
    size_t end = json.find('\"', pos);
    if (end == std::string::npos) return false;

    value = json.substr(pos, end - pos);
    pos = end + 1;
    return true;
}

bool SceneLoader::ReadFloat(const std::string& json, size_t& pos, float& value)
{
    SkipWhitespace(json, pos);

    size_t start = pos;
    while (pos < json.length() && (isdigit(json[pos]) || json[pos] == '.' || json[pos] == '-' || json[pos] == 'e' || json[pos] == 'E'))
    {
        pos++;
    }

    if (pos > start)
    {
        std::string numStr = json.substr(start, pos - start);
        try
        {
            value = std::stof(numStr);
        }
        catch (const std::invalid_argument&)
        {
            m_lastError = "Invalid float format: " + numStr;
            return false;
        }
        catch (const std::out_of_range&)
        {
            m_lastError = "Float out of range: " + numStr;
            return false;
        }
        SkipWhitespace(json, pos);
        if (pos < json.length() && json[pos] == ',') pos++;
        return true;
    }

    return false;
}

bool SceneLoader::ReadInt(const std::string& json, size_t& pos, int& value)
{
    float f;
    if (ReadFloat(json, pos, f))
    {
        value = static_cast<int>(f);
        return true;
    }
    return false;
}

bool SceneLoader::ReadBool(const std::string& json, size_t& pos, bool& value)
{
    SkipWhitespace(json, pos);

    if (json.substr(pos, 4) == "true")
    {
        value = true;
        pos += 4;
        return true;
    }
    else if (json.substr(pos, 5) == "false")
    {
        value = false;
        pos += 5;
        return true;
    }

    return false;
}

void SceneLoader::SkipWhitespace(const std::string& json, size_t& pos)
{
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n' || json[pos] == '\r'))
    {
        pos++;
    }
}

bool SceneLoader::SaveToJSON(const std::string& filename, PxScene* scene)
{
    // TODO: Implement scene export to JSON
    // For now, return a basic implementation
    m_lastError = "SaveToJSON not yet implemented";
    return false;
}
