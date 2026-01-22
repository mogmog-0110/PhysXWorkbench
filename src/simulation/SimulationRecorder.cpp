#include "SimulationRecorder.h"
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace physx;

SimulationRecorder::SimulationRecorder()
    : m_isRecording(false)
    , m_isPaused(false)
    , m_currentFrameNumber(0)
{
}

SimulationRecorder::~SimulationRecorder()
{
}

void SimulationRecorder::StartRecording()
{
    m_isRecording = true;
    m_isPaused = false;
    m_currentFrameNumber = 0;
    m_frames.clear();
    std::cout << "Recording started" << std::endl;
}

void SimulationRecorder::StopRecording()
{
    m_isRecording = false;
    m_isPaused = false;
    std::cout << "Recording stopped. Total frames: " << m_frames.size() << std::endl;
}

void SimulationRecorder::PauseRecording()
{
    m_isPaused = true;
    std::cout << "Recording paused" << std::endl;
}

void SimulationRecorder::ResumeRecording()
{
    m_isPaused = false;
    std::cout << "Recording resumed" << std::endl;
}

void SimulationRecorder::RecordFrame(PxScene* scene, float timeStamp)
{
    if (!IsRecording() || !scene)
        return;

    FrameSnapshot frame;
    frame.frameNumber = m_currentFrameNumber++;
    frame.timeStamp = timeStamp;

    // Get all actors
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
    if (nbActors > 0)
    {
        std::vector<PxActor*> actors(nbActors);
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC,
                        &actors[0], nbActors);

        for (PxU32 i = 0; i < nbActors; i++)
        {
            PxRigidActor* rigidActor = actors[i]->is<PxRigidActor>();
            if (!rigidActor)
                continue;

            ActorFrameData actorData;
            actorData.actorID = GetActorID(actors[i]);

            PxTransform transform = rigidActor->getGlobalPose();
            actorData.posX = transform.p.x;
            actorData.posY = transform.p.y;
            actorData.posZ = transform.p.z;
            actorData.quatW = transform.q.w;
            actorData.quatX = transform.q.x;
            actorData.quatY = transform.q.y;
            actorData.quatZ = transform.q.z;

            // Get velocity for dynamic actors
            PxRigidDynamic* dynamicActor = actors[i]->is<PxRigidDynamic>();
            if (dynamicActor)
            {
                PxVec3 vel = dynamicActor->getLinearVelocity();
                PxVec3 angVel = dynamicActor->getAngularVelocity();

                actorData.velX = vel.x;
                actorData.velY = vel.y;
                actorData.velZ = vel.z;
                actorData.angVelX = angVel.x;
                actorData.angVelY = angVel.y;
                actorData.angVelZ = angVel.z;
            }
            else
            {
                actorData.velX = actorData.velY = actorData.velZ = 0.0f;
                actorData.angVelX = actorData.angVelY = actorData.angVelZ = 0.0f;
            }

            frame.actors.push_back(actorData);
        }
    }

    m_frames.push_back(frame);
}

void SimulationRecorder::RecordCollision(PxActor* actor1, PxActor* actor2,
                                        const PxVec3& contactPos, float impulse)
{
    if (!IsRecording() || m_frames.empty())
        return;

    CollisionData collision;
    collision.actor1ID = GetActorID(actor1);
    collision.actor2ID = GetActorID(actor2);
    collision.contactPosX = contactPos.x;
    collision.contactPosY = contactPos.y;
    collision.contactPosZ = contactPos.z;
    collision.impulse = impulse;

    m_frames.back().collisions.push_back(collision);
}

bool SimulationRecorder::ExportToCSV(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    // Write header
    file << "Frame,Time,ActorID,PosX,PosY,PosZ,VelX,VelY,VelZ,AngVelX,AngVelY,AngVelZ,QuatW,QuatX,QuatY,QuatZ\n";

    // Write data
    for (const auto& frame : m_frames)
    {
        for (const auto& actor : frame.actors)
        {
            file << frame.frameNumber << ","
                 << std::fixed << std::setprecision(6) << frame.timeStamp << ","
                 << actor.actorID << ","
                 << actor.posX << "," << actor.posY << "," << actor.posZ << ","
                 << actor.velX << "," << actor.velY << "," << actor.velZ << ","
                 << actor.angVelX << "," << actor.angVelY << "," << actor.angVelZ << ","
                 << actor.quatW << "," << actor.quatX << "," << actor.quatY << "," << actor.quatZ << "\n";
        }
    }

    // Write collisions to separate file
    std::string collisionFilename = filename.substr(0, filename.find_last_of('.')) + "_collisions.csv";
    std::ofstream collFile(collisionFilename);
    if (collFile.is_open())
    {
        collFile << "Frame,Time,Actor1ID,Actor2ID,ContactX,ContactY,ContactZ,Impulse\n";

        for (const auto& frame : m_frames)
        {
            for (const auto& collision : frame.collisions)
            {
                collFile << frame.frameNumber << ","
                        << std::fixed << std::setprecision(6) << frame.timeStamp << ","
                        << collision.actor1ID << "," << collision.actor2ID << ","
                        << collision.contactPosX << "," << collision.contactPosY << "," << collision.contactPosZ << ","
                        << collision.impulse << "\n";
            }
        }
        collFile.close();
    }

    file.close();
    std::cout << "Exported " << m_frames.size() << " frames to " << filename << std::endl;
    return true;
}

bool SimulationRecorder::ExportToJSON(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    file << "{\n  \"frames\": [\n";

    for (size_t f = 0; f < m_frames.size(); f++)
    {
        const auto& frame = m_frames[f];
        file << "    {\n";
        file << "      \"frameNumber\": " << frame.frameNumber << ",\n";
        file << "      \"timeStamp\": " << std::fixed << std::setprecision(6) << frame.timeStamp << ",\n";
        file << "      \"actors\": [\n";

        for (size_t a = 0; a < frame.actors.size(); a++)
        {
            const auto& actor = frame.actors[a];
            file << "        {\n";
            file << "          \"id\": " << actor.actorID << ",\n";
            file << "          \"position\": [" << actor.posX << ", " << actor.posY << ", " << actor.posZ << "],\n";
            file << "          \"velocity\": [" << actor.velX << ", " << actor.velY << ", " << actor.velZ << "],\n";
            file << "          \"angularVelocity\": [" << actor.angVelX << ", " << actor.angVelY << ", " << actor.angVelZ << "],\n";
            file << "          \"quaternion\": [" << actor.quatW << ", " << actor.quatX << ", " << actor.quatY << ", " << actor.quatZ << "]\n";
            file << "        }" << (a < frame.actors.size() - 1 ? "," : "") << "\n";
        }

        file << "      ],\n";
        file << "      \"collisions\": [\n";

        for (size_t c = 0; c < frame.collisions.size(); c++)
        {
            const auto& coll = frame.collisions[c];
            file << "        {\n";
            file << "          \"actor1\": " << coll.actor1ID << ",\n";
            file << "          \"actor2\": " << coll.actor2ID << ",\n";
            file << "          \"contactPos\": [" << coll.contactPosX << ", " << coll.contactPosY << ", " << coll.contactPosZ << "],\n";
            file << "          \"impulse\": " << coll.impulse << "\n";
            file << "        }" << (c < frame.collisions.size() - 1 ? "," : "") << "\n";
        }

        file << "      ]\n";
        file << "    }" << (f < m_frames.size() - 1 ? "," : "") << "\n";
    }

    file << "  ]\n}\n";
    file.close();

    std::cout << "Exported " << m_frames.size() << " frames to " << filename << std::endl;
    return true;
}

void SimulationRecorder::Clear()
{
    m_frames.clear();
    m_currentFrameNumber = 0;
}

size_t SimulationRecorder::GetTotalCollisions() const
{
    size_t total = 0;
    for (const auto& frame : m_frames)
    {
        total += frame.collisions.size();
    }
    return total;
}

float SimulationRecorder::GetRecordingDuration() const
{
    if (m_frames.empty())
        return 0.0f;
    return m_frames.back().timeStamp - m_frames.front().timeStamp;
}

uint32_t SimulationRecorder::GetActorID(PxActor* actor)
{
    // Use the actor's pointer as a simple unique ID
    // In a real application, you might want to use userData or a hash map
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(actor) & 0xFFFFFFFF);
}

std::string SimulationRecorder::Vector3ToString(float x, float y, float z)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "(" << x << ", " << y << ", " << z << ")";
    return oss.str();
}
