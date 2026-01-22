#include "ExperimentController.h"
#include <iostream>
#include <algorithm>

using namespace physx;

ExperimentController::ExperimentController()
    : m_resetRequested(false)
    , m_stepMode(false)
    , m_stepRequested(false)
{
    InitializeDefaultPresets();
}

ExperimentController::~ExperimentController()
{
}

void ExperimentController::SetGravity(const PxVec3& gravity)
{
    m_params.gravity = gravity;
    std::cout << "Gravity set to: (" << gravity.x << ", " << gravity.y << ", " << gravity.z << ")" << std::endl;
}

void ExperimentController::SetFriction(float friction)
{
    m_params.friction = PxClamp(friction, 0.0f, 1.0f);
    std::cout << "Friction set to: " << m_params.friction << std::endl;
}

void ExperimentController::SetRestitution(float restitution)
{
    m_params.restitution = PxClamp(restitution, 0.0f, 1.0f);
    std::cout << "Restitution set to: " << m_params.restitution << std::endl;
}

void ExperimentController::SetDensity(float density)
{
    m_params.density = PxMax(density, 0.001f);
    std::cout << "Density set to: " << m_params.density << std::endl;
}

void ExperimentController::ApplyParametersToScene(PxScene* scene, PxMaterial* material)
{
    if (!scene)
        return;

    // Update gravity
    scene->setGravity(m_params.gravity);

    // Update material properties
    if (material)
    {
        material->setStaticFriction(m_params.friction);
        material->setDynamicFriction(m_params.friction);
        material->setRestitution(m_params.restitution);
    }

    std::cout << "Applied parameters to scene" << std::endl;
}

void ExperimentController::Pause()
{
    m_params.paused = true;
    std::cout << "Simulation paused" << std::endl;
}

void ExperimentController::Resume()
{
    m_params.paused = false;
    std::cout << "Simulation resumed" << std::endl;
}

void ExperimentController::TogglePause()
{
    m_params.paused = !m_params.paused;
    std::cout << "Simulation " << (m_params.paused ? "paused" : "resumed") << std::endl;
}

void ExperimentController::SetTimeScale(float scale)
{
    m_params.timeScale = PxClamp(scale, 0.0f, 10.0f);
    std::cout << "Time scale set to: " << m_params.timeScale << "x" << std::endl;
}

void ExperimentController::StepForward()
{
    if (m_stepMode)
    {
        m_stepRequested = true;
    }
}

float ExperimentController::GetAdjustedTimestep(float baseTimestep) const
{
    if (m_params.paused && !m_stepRequested)
        return 0.0f;

    return baseTimestep * m_params.timeScale;
}

void ExperimentController::LoadPreset(const std::string& presetName)
{
    auto it = std::find_if(m_presets.begin(), m_presets.end(),
        [&presetName](const auto& preset) { return preset.first == presetName; });

    if (it != m_presets.end())
    {
        m_params = it->second;
        std::cout << "Loaded preset: " << presetName << std::endl;
    }
    else
    {
        std::cerr << "Preset not found: " << presetName << std::endl;
    }
}

void ExperimentController::SavePreset(const std::string& presetName)
{
    auto it = std::find_if(m_presets.begin(), m_presets.end(),
        [&presetName](const auto& preset) { return preset.first == presetName; });

    if (it != m_presets.end())
    {
        it->second = m_params;
    }
    else
    {
        m_presets.push_back({ presetName, m_params });
    }

    std::cout << "Saved preset: " << presetName << std::endl;
}

std::vector<std::string> ExperimentController::GetPresetNames() const
{
    std::vector<std::string> names;
    for (const auto& preset : m_presets)
    {
        names.push_back(preset.first);
    }
    return names;
}

void ExperimentController::InitializeDefaultPresets()
{
    // Earth gravity preset
    SimulationParameters earth;
    earth.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    earth.friction = 0.5f;
    earth.restitution = 0.6f;
    earth.density = 1.0f;
    earth.timeScale = 1.0f;
    m_presets.push_back({ "Earth", earth });

    // Moon gravity preset
    SimulationParameters moon;
    moon.gravity = PxVec3(0.0f, -1.62f, 0.0f);
    moon.friction = 0.5f;
    moon.restitution = 0.6f;
    moon.density = 1.0f;
    moon.timeScale = 1.0f;
    m_presets.push_back({ "Moon", moon });

    // Zero gravity preset
    SimulationParameters zeroG;
    zeroG.gravity = PxVec3(0.0f, 0.0f, 0.0f);
    zeroG.friction = 0.5f;
    zeroG.restitution = 0.6f;
    zeroG.density = 1.0f;
    zeroG.timeScale = 1.0f;
    m_presets.push_back({ "Zero Gravity", zeroG });

    // Bouncy preset
    SimulationParameters bouncy;
    bouncy.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    bouncy.friction = 0.1f;
    bouncy.restitution = 0.95f;
    bouncy.density = 0.5f;
    bouncy.timeScale = 1.0f;
    m_presets.push_back({ "Bouncy", bouncy });

    // Sticky preset
    SimulationParameters sticky;
    sticky.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    sticky.friction = 0.9f;
    sticky.restitution = 0.1f;
    sticky.density = 2.0f;
    sticky.timeScale = 1.0f;
    m_presets.push_back({ "Sticky", sticky });

    // Set default to Earth
    m_params = earth;
}
