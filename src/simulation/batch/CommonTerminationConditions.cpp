#include "CommonTerminationConditions.h"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <numeric>

namespace batch
{

// =============================================================================
// TimeoutCondition
// =============================================================================

TimeoutCondition::TimeoutCondition(float maxTime)
    : m_maxTime(maxTime)
{
}

std::string TimeoutCondition::getDescription() const
{
    return "Terminates after " + std::to_string(m_maxTime) + " seconds";
}

bool TimeoutCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    (void)scene;
    return time >= m_maxTime;
}

void TimeoutCondition::reset()
{
    // Nothing to reset
}

std::unique_ptr<ITerminationCondition> TimeoutCondition::clone() const
{
    return std::make_unique<TimeoutCondition>(m_maxTime);
}

// =============================================================================
// BondCountCondition
// =============================================================================

BondCountCondition::BondCountCondition(
    bonding::DynamicBondManager* bondManager,
    int targetCount,
    const std::string& condition)
    : m_bondManager(bondManager)
    , m_targetCount(targetCount)
    , m_condition(condition)
{
}

std::string BondCountCondition::getDescription() const
{
    return "Terminates when bond count " + m_condition + " " + std::to_string(m_targetCount);
}

bool BondCountCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    (void)scene;
    (void)time;

    if (!m_bondManager)
        return false;

    int currentCount = static_cast<int>(m_bondManager->getBondCount());

    if (m_condition == ">=")
        return currentCount >= m_targetCount;
    else if (m_condition == "<=")
        return currentCount <= m_targetCount;
    else if (m_condition == "==")
        return currentCount == m_targetCount;

    return false;
}

void BondCountCondition::reset()
{
    // Nothing to reset
}

std::unique_ptr<ITerminationCondition> BondCountCondition::clone() const
{
    return std::make_unique<BondCountCondition>(m_bondManager, m_targetCount, m_condition);
}

// =============================================================================
// SteadyStateCondition
// =============================================================================

SteadyStateCondition::SteadyStateCondition(float energyThreshold, float holdTime)
    : m_energyThreshold(energyThreshold)
    , m_holdTime(holdTime)
{
}

std::string SteadyStateCondition::getDescription() const
{
    return "Terminates when kinetic energy < " + std::to_string(m_energyThreshold) +
           " for " + std::to_string(m_holdTime) + " seconds";
}

bool SteadyStateCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    if (!scene)
        return false;

    // Compute kinetic energy
    float totalEnergy = 0.0f;

    physx::PxU32 nbActors = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
    std::vector<physx::PxRigidActor*> actors(nbActors);
    scene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC,
                     reinterpret_cast<physx::PxActor**>(actors.data()), nbActors);

    for (auto* actor : actors)
    {
        auto* dynamic = static_cast<physx::PxRigidDynamic*>(actor);
        if (!dynamic)
            continue;

        float mass = dynamic->getMass();
        physx::PxVec3 vel = dynamic->getLinearVelocity();
        totalEnergy += 0.5f * mass * vel.magnitudeSquared();
    }

    float dt = time - m_lastTime;
    m_lastTime = time;

    if (totalEnergy < m_energyThreshold)
    {
        m_timeBelowThreshold += dt;
    }
    else
    {
        m_timeBelowThreshold = 0.0f;
    }

    return m_timeBelowThreshold >= m_holdTime;
}

void SteadyStateCondition::reset()
{
    m_timeBelowThreshold = 0.0f;
    m_lastTime = 0.0f;
}

std::unique_ptr<ITerminationCondition> SteadyStateCondition::clone() const
{
    return std::make_unique<SteadyStateCondition>(m_energyThreshold, m_holdTime);
}

// =============================================================================
// RingFormationCondition
// =============================================================================

RingFormationCondition::RingFormationCondition(
    bonding::DynamicBondManager* bondManager,
    int targetRingSize)
    : m_bondManager(bondManager)
    , m_targetRingSize(targetRingSize)
{
}

std::string RingFormationCondition::getDescription() const
{
    if (m_targetRingSize > 0)
        return "Terminates when ring of size " + std::to_string(m_targetRingSize) + " forms";
    return "Terminates when any ring forms";
}

bool RingFormationCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    (void)scene;
    (void)time;

    int ringSize = 0;
    return detectRing(ringSize);
}

void RingFormationCondition::reset()
{
    // Nothing to reset
}

std::unique_ptr<ITerminationCondition> RingFormationCondition::clone() const
{
    return std::make_unique<RingFormationCondition>(m_bondManager, m_targetRingSize);
}

bool RingFormationCondition::detectRing(int& ringSize) const
{
    if (!m_bondManager)
        return false;

    const auto& bonds = m_bondManager->getBonds();

    // Build adjacency list
    std::unordered_map<uint64_t, std::vector<uint64_t>> adjacency;
    for (const auto& [bondId, bond] : bonds)
    {
        adjacency[bond.endpoint1.entityId].push_back(bond.endpoint2.entityId);
        adjacency[bond.endpoint2.entityId].push_back(bond.endpoint1.entityId);
    }

    // DFS to find cycles
    std::unordered_set<uint64_t> visited;
    std::unordered_map<uint64_t, int> depth;

    std::function<bool(uint64_t, uint64_t, int)> dfs = [&](uint64_t node, uint64_t parent, int d) -> bool
    {
        if (visited.count(node))
        {
            int cycleSize = d - depth[node];
            if (m_targetRingSize == 0 || cycleSize == m_targetRingSize)
            {
                ringSize = cycleSize;
                return true;
            }
            return false;
        }

        visited.insert(node);
        depth[node] = d;

        for (uint64_t neighbor : adjacency[node])
        {
            if (neighbor != parent)
            {
                if (dfs(neighbor, node, d + 1))
                    return true;
            }
        }

        return false;
    };

    for (const auto& [entityId, neighbors] : adjacency)
    {
        if (!visited.count(entityId))
        {
            if (dfs(entityId, 0, 0))
                return true;
        }
    }

    return false;
}

// =============================================================================
// AllSaturatedCondition
// =============================================================================

AllSaturatedCondition::AllSaturatedCondition(bonding::DynamicBondManager* bondManager)
    : m_bondManager(bondManager)
{
}

std::string AllSaturatedCondition::getDescription() const
{
    return "Terminates when all entities have saturated bonding sites";
}

bool AllSaturatedCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    (void)scene;
    (void)time;

    if (!m_bondManager)
        return false;

    auto stats = m_bondManager->getStats();
    return stats.saturatedEntityCount == stats.entityCount && stats.entityCount > 0;
}

void AllSaturatedCondition::reset()
{
    // Nothing to reset
}

std::unique_ptr<ITerminationCondition> AllSaturatedCondition::clone() const
{
    return std::make_unique<AllSaturatedCondition>(m_bondManager);
}

// =============================================================================
// CustomCondition
// =============================================================================

CustomCondition::CustomCondition(
    const std::string& name,
    CheckFunc checkFunc,
    const std::string& description)
    : m_name(name)
    , m_description(description)
    , m_checkFunc(std::move(checkFunc))
{
}

bool CustomCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    if (m_checkFunc)
    {
        return m_checkFunc(scene, time);
    }
    return false;
}

void CustomCondition::reset()
{
    // Nothing to reset
}

std::unique_ptr<ITerminationCondition> CustomCondition::clone() const
{
    return std::make_unique<CustomCondition>(m_name, m_checkFunc, m_description);
}

// =============================================================================
// MetricThresholdCondition
// =============================================================================

MetricThresholdCondition::MetricThresholdCondition(
    MetricPtr metric,
    float threshold,
    Comparison comparison)
    : m_metric(std::move(metric))
    , m_threshold(threshold)
    , m_comparison(comparison)
{
}

std::string MetricThresholdCondition::getDescription() const
{
    std::string op;
    switch (m_comparison)
    {
    case Comparison::LESS: op = "<"; break;
    case Comparison::LESS_EQUAL: op = "<="; break;
    case Comparison::EQUAL: op = "=="; break;
    case Comparison::GREATER_EQUAL: op = ">="; break;
    case Comparison::GREATER: op = ">"; break;
    }
    return "Terminates when " + m_metric->getName() + " " + op + " " + std::to_string(m_threshold);
}

bool MetricThresholdCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    (void)scene;
    (void)time;

    if (!m_metric)
        return false;

    MetricValue value = m_metric->getValue();

    // Extract float value
    float floatValue = 0.0f;
    if (auto* f = std::get_if<float>(&value))
        floatValue = *f;
    else if (auto* d = std::get_if<double>(&value))
        floatValue = static_cast<float>(*d);
    else if (auto* i = std::get_if<int>(&value))
        floatValue = static_cast<float>(*i);
    else
        return false; // Can't compare non-numeric

    switch (m_comparison)
    {
    case Comparison::LESS: return floatValue < m_threshold;
    case Comparison::LESS_EQUAL: return floatValue <= m_threshold;
    case Comparison::EQUAL: return std::abs(floatValue - m_threshold) < 0.0001f;
    case Comparison::GREATER_EQUAL: return floatValue >= m_threshold;
    case Comparison::GREATER: return floatValue > m_threshold;
    }

    return false;
}

void MetricThresholdCondition::reset()
{
    if (m_metric)
        m_metric->reset();
}

std::unique_ptr<ITerminationCondition> MetricThresholdCondition::clone() const
{
    return std::make_unique<MetricThresholdCondition>(m_metric, m_threshold, m_comparison);
}

// =============================================================================
// CompositeCondition
// =============================================================================

CompositeCondition::CompositeCondition(Logic logic)
    : m_logic(logic)
{
}

void CompositeCondition::addCondition(TerminationConditionPtr condition)
{
    m_conditions.push_back(std::move(condition));
}

void CompositeCondition::removeCondition(const std::string& name)
{
    m_conditions.erase(
        std::remove_if(m_conditions.begin(), m_conditions.end(),
            [&name](const TerminationConditionPtr& cond)
            {
                return cond->getName() == name;
            }),
        m_conditions.end());
}

void CompositeCondition::clearConditions()
{
    m_conditions.clear();
}

std::string CompositeCondition::getDescription() const
{
    std::string logic = (m_logic == Logic::AND) ? "AND" : "OR";
    std::string desc = "Composite condition (" + logic + "): ";
    for (size_t i = 0; i < m_conditions.size(); ++i)
    {
        if (i > 0)
            desc += ", ";
        desc += m_conditions[i]->getName();
    }
    return desc;
}

bool CompositeCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    if (m_conditions.empty())
        return false;

    if (m_logic == Logic::AND)
    {
        for (const auto& cond : m_conditions)
        {
            if (!cond->shouldTerminate(scene, time))
                return false;
        }
        return true;
    }
    else // OR
    {
        for (const auto& cond : m_conditions)
        {
            if (cond->shouldTerminate(scene, time))
                return true;
        }
        return false;
    }
}

void CompositeCondition::reset()
{
    for (auto& cond : m_conditions)
    {
        cond->reset();
    }
}

std::unique_ptr<ITerminationCondition> CompositeCondition::clone() const
{
    auto clone = std::make_unique<CompositeCondition>(m_logic);
    for (const auto& cond : m_conditions)
    {
        clone->addCondition(std::shared_ptr<ITerminationCondition>(cond->clone()));
    }
    return clone;
}

// =============================================================================
// MovingAverageSteadyStateCondition
// =============================================================================

MovingAverageSteadyStateCondition::MovingAverageSteadyStateCondition(
    size_t windowSize,
    float varianceThreshold,
    float holdTime)
    : m_windowSize(windowSize)
    , m_varianceThreshold(varianceThreshold)
    , m_holdTime(holdTime)
{
}

std::string MovingAverageSteadyStateCondition::getDescription() const
{
    return "Terminates when kinetic energy variance < " + std::to_string(m_varianceThreshold) +
           " over " + std::to_string(m_windowSize) + " samples for " +
           std::to_string(m_holdTime) + " seconds";
}

bool MovingAverageSteadyStateCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    // Compute current kinetic energy
    float energy = computeKineticEnergy(scene);

    // Add to window
    m_energyWindow.push_back(energy);
    while (m_energyWindow.size() > m_windowSize)
    {
        m_energyWindow.pop_front();
    }

    float dt = time - m_lastTime;
    m_lastTime = time;

    // Need full window to compute variance
    if (m_energyWindow.size() < m_windowSize)
        return false;

    float variance = computeVariance();

    if (variance < m_varianceThreshold)
    {
        m_timeBelowThreshold += dt;
    }
    else
    {
        m_timeBelowThreshold = 0.0f;
    }

    return m_timeBelowThreshold >= m_holdTime;
}

void MovingAverageSteadyStateCondition::reset()
{
    m_energyWindow.clear();
    m_timeBelowThreshold = 0.0f;
    m_lastTime = 0.0f;
}

std::unique_ptr<ITerminationCondition> MovingAverageSteadyStateCondition::clone() const
{
    return std::make_unique<MovingAverageSteadyStateCondition>(
        m_windowSize, m_varianceThreshold, m_holdTime);
}

float MovingAverageSteadyStateCondition::computeKineticEnergy(physx::PxScene* scene) const
{
    if (!scene)
        return 0.0f;

    float totalEnergy = 0.0f;

    physx::PxU32 nbActors = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
    std::vector<physx::PxRigidActor*> actors(nbActors);
    scene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC,
                     reinterpret_cast<physx::PxActor**>(actors.data()), nbActors);

    for (auto* actor : actors)
    {
        auto* dynamic = static_cast<physx::PxRigidDynamic*>(actor);
        if (!dynamic)
            continue;

        float mass = dynamic->getMass();
        physx::PxVec3 vel = dynamic->getLinearVelocity();
        totalEnergy += 0.5f * mass * vel.magnitudeSquared();
    }

    return totalEnergy;
}

float MovingAverageSteadyStateCondition::computeVariance() const
{
    if (m_energyWindow.empty())
        return 0.0f;

    float mean = std::accumulate(m_energyWindow.begin(), m_energyWindow.end(), 0.0f) /
                 static_cast<float>(m_energyWindow.size());

    float variance = 0.0f;
    for (float val : m_energyWindow)
    {
        float diff = val - mean;
        variance += diff * diff;
    }
    variance /= static_cast<float>(m_energyWindow.size());

    return variance;
}

} // namespace batch
