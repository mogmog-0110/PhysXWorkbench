#pragma once

#include <PxPhysicsAPI.h>
#include <vector>
#include <string>
#include <fstream>
#include <memory>

// Note: Avoid 'using namespace' in headers - use explicit physx:: prefix instead

// Structure to hold frame data for a single actor
struct ActorFrameData
{
    uint32_t actorID;
    float posX, posY, posZ;
    float velX, velY, velZ;
    float angVelX, angVelY, angVelZ;
    float quatW, quatX, quatY, quatZ;
};

// Structure to hold collision data
struct CollisionData
{
    uint32_t actor1ID;
    uint32_t actor2ID;
    float contactPosX, contactPosY, contactPosZ;
    float impulse;
};

// Structure to hold complete frame snapshot
struct FrameSnapshot
{
    uint32_t frameNumber;
    float timeStamp;
    std::vector<ActorFrameData> actors;
    std::vector<CollisionData> collisions;
};

class SimulationRecorder
{
public:
    SimulationRecorder();
    ~SimulationRecorder();

    // Recording control
    void StartRecording();
    void StopRecording();
    void PauseRecording();
    void ResumeRecording();
    bool IsRecording() const { return m_isRecording && !m_isPaused; }
    bool IsPaused() const { return m_isPaused; }

    // Data recording
    void RecordFrame(physx::PxScene* scene, float timeStamp);
    void RecordCollision(physx::PxActor* actor1, physx::PxActor* actor2, const physx::PxVec3& contactPos, float impulse);

    // Data export
    bool ExportToCSV(const std::string& filename);
    bool ExportToJSON(const std::string& filename);

    // Data access
    const std::vector<FrameSnapshot>& GetFrames() const { return m_frames; }
    void Clear();

    // Statistics
    size_t GetFrameCount() const { return m_frames.size(); }
    size_t GetTotalCollisions() const;
    float GetRecordingDuration() const;

private:
    bool m_isRecording;
    bool m_isPaused;
    std::vector<FrameSnapshot> m_frames;
    uint32_t m_currentFrameNumber;

    // Helper functions
    uint32_t GetActorID(physx::PxActor* actor);
    std::string Vector3ToString(float x, float y, float z);
};
