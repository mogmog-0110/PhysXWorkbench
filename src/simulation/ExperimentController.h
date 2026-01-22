#pragma once

#include <PxPhysicsAPI.h>
#include <functional>
#include <vector>
#include <string>

// Note: Avoid 'using namespace' in headers - use explicit physx:: prefix instead

// Structure to hold simulation parameters
struct SimulationParameters
{
    physx::PxVec3 gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    float friction = 0.5f;
    float restitution = 0.6f;
    float density = 1.0f;
    float timeScale = 1.0f;
    bool paused = false;
};

// Callback type for scene reset
using SceneResetCallback = std::function<void()>;

class ExperimentController
{
public:
    ExperimentController();
    ~ExperimentController();

    // Parameter control
    void SetGravity(const physx::PxVec3& gravity);
    void SetFriction(float friction);
    void SetRestitution(float restitution);
    void SetDensity(float density);

    // Apply parameters to scene
    void ApplyParametersToScene(physx::PxScene* scene, physx::PxMaterial* material);

    // Time control
    void Pause();
    void Resume();
    void TogglePause();
    void SetTimeScale(float scale);  // 0.1 = slow motion, 2.0 = fast forward
    float GetTimeScale() const { return m_params.timeScale; }
    bool IsPaused() const { return m_params.paused; }

    // Step-by-step control
    void EnableStepMode(bool enable) { m_stepMode = enable; }
    void StepForward();  // Advance one frame in step mode
    bool IsStepMode() const { return m_stepMode; }
    bool CanStep() const { return m_stepRequested; }
    void ClearStepRequest() { m_stepRequested = false; }

    // Scene reset
    void SetResetCallback(SceneResetCallback callback) { m_resetCallback = callback; }
    void RequestReset() { m_resetRequested = true; }
    bool IsResetRequested() const { return m_resetRequested; }
    void ClearResetRequest() { m_resetRequested = false; }

    // Parameter access
    const SimulationParameters& GetParameters() const { return m_params; }
    SimulationParameters& GetParameters() { return m_params; }

    // Calculate adjusted timestep based on time scale
    float GetAdjustedTimestep(float baseTimestep) const;

    // Presets
    void LoadPreset(const std::string& presetName);
    void SavePreset(const std::string& presetName);
    std::vector<std::string> GetPresetNames() const;

private:
    SimulationParameters m_params;
    SceneResetCallback m_resetCallback;
    bool m_resetRequested;

    bool m_stepMode;
    bool m_stepRequested;

    // Presets storage
    std::vector<std::pair<std::string, SimulationParameters>> m_presets;
    void InitializeDefaultPresets();
};
