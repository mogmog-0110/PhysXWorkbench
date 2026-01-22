#pragma once

#include "ITerminationCondition.h"
#include "IMetric.h"
#include "../bonding/DynamicBondManager.h"
#include <functional>
#include <deque>

namespace batch
{

/// Condition: Terminate after a specified time
class TimeoutCondition : public ITerminationCondition
{
public:
    /// @param maxTime Maximum simulation time in seconds
    explicit TimeoutCondition(float maxTime = 60.0f);

    std::string getName() const override { return "timeout"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

    float getMaxTime() const { return m_maxTime; }
    void setMaxTime(float maxTime) { m_maxTime = maxTime; }

private:
    float m_maxTime;
};

/// Condition: Terminate when bond count reaches a target
class BondCountCondition : public ITerminationCondition
{
public:
    /// @param bondManager Reference to the bond manager
    /// @param targetCount Target number of bonds
    /// @param condition ">=", "<=", or "==" for comparison
    BondCountCondition(
        bonding::DynamicBondManager* bondManager,
        int targetCount,
        const std::string& condition = ">=");

    std::string getName() const override { return "bond_count"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    bonding::DynamicBondManager* m_bondManager;
    int m_targetCount;
    std::string m_condition;
};

/// Condition: Terminate when kinetic energy falls below threshold
/// Indicates the system has reached a steady state
class SteadyStateCondition : public ITerminationCondition
{
public:
    /// @param energyThreshold Maximum kinetic energy for steady state
    /// @param holdTime Time that energy must stay below threshold
    explicit SteadyStateCondition(float energyThreshold = 0.1f, float holdTime = 1.0f);

    std::string getName() const override { return "steady_state"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    float m_energyThreshold;
    float m_holdTime;
    mutable float m_timeBelowThreshold = 0.0f;
    mutable float m_lastTime = 0.0f;
};

/// Condition: Terminate when a ring is formed
class RingFormationCondition : public ITerminationCondition
{
public:
    /// @param bondManager Reference to the bond manager
    /// @param targetRingSize Size of ring to detect (0 = any size)
    explicit RingFormationCondition(
        bonding::DynamicBondManager* bondManager,
        int targetRingSize = 0);

    std::string getName() const override { return "ring_formation"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    bonding::DynamicBondManager* m_bondManager;
    int m_targetRingSize;

    bool detectRing(int& ringSize) const;
};

/// Condition: Terminate when all entities are saturated
class AllSaturatedCondition : public ITerminationCondition
{
public:
    /// @param bondManager Reference to the bond manager
    explicit AllSaturatedCondition(bonding::DynamicBondManager* bondManager);

    std::string getName() const override { return "all_saturated"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    bonding::DynamicBondManager* m_bondManager;
};

/// Condition: Custom condition using a lambda function
class CustomCondition : public ITerminationCondition
{
public:
    using CheckFunc = std::function<bool(physx::PxScene*, float)>;

    /// @param name Name of the condition
    /// @param checkFunc Function to check termination
    /// @param description Optional description
    CustomCondition(
        const std::string& name,
        CheckFunc checkFunc,
        const std::string& description = "");

    std::string getName() const override { return m_name; }
    std::string getDescription() const override { return m_description; }

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    std::string m_name;
    std::string m_description;
    CheckFunc m_checkFunc;
};

/// Condition: Terminate when a metric reaches a target value
class MetricThresholdCondition : public ITerminationCondition
{
public:
    enum class Comparison { LESS, LESS_EQUAL, EQUAL, GREATER_EQUAL, GREATER };

    /// @param metric The metric to monitor
    /// @param threshold Target value
    /// @param comparison How to compare metric to threshold
    MetricThresholdCondition(
        MetricPtr metric,
        float threshold,
        Comparison comparison = Comparison::GREATER_EQUAL);

    std::string getName() const override { return "metric_threshold"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    MetricPtr m_metric;
    float m_threshold;
    Comparison m_comparison;
};

/// Condition: Combine multiple conditions with AND or OR logic
class CompositeCondition : public ITerminationCondition
{
public:
    enum class Logic { AND, OR };

    /// @param logic How to combine conditions
    explicit CompositeCondition(Logic logic = Logic::OR);

    void addCondition(TerminationConditionPtr condition);
    void removeCondition(const std::string& name);
    void clearConditions();

    std::string getName() const override { return "composite"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    Logic m_logic;
    std::vector<TerminationConditionPtr> m_conditions;
};

/// Condition: Moving average based steady state detection
/// More robust than simple threshold for noisy systems
class MovingAverageSteadyStateCondition : public ITerminationCondition
{
public:
    /// @param windowSize Number of samples to average
    /// @param varianceThreshold Maximum variance for steady state
    /// @param holdTime Time to maintain steady state before terminating
    explicit MovingAverageSteadyStateCondition(
        size_t windowSize = 60,
        float varianceThreshold = 0.01f,
        float holdTime = 1.0f);

    std::string getName() const override { return "moving_average_steady_state"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

private:
    size_t m_windowSize;
    float m_varianceThreshold;
    float m_holdTime;

    mutable std::deque<float> m_energyWindow;
    mutable float m_timeBelowThreshold = 0.0f;
    mutable float m_lastTime = 0.0f;

    float computeKineticEnergy(physx::PxScene* scene) const;
    float computeVariance() const;
};

} // namespace batch
