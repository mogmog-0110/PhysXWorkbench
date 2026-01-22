#pragma once

#include "ITerminationCondition.h"
#include "IMetric.h"
#include "CommonMetrics.h"
#include <memory>
#include <deque>

// Note: This file provides an MSER-based steady state detection condition.
// To use the full MSER library, ensure mser-cpp is available as a submodule
// or installed library. This implementation provides a simplified version
// that doesn't require the external library.

namespace batch
{

/// Configuration for MSER-based steady state detection
struct MSERConfig
{
    /// Minimum number of samples before checking for convergence
    size_t minSamples = 100;

    /// Interval between convergence checks (in samples)
    size_t checkInterval = 10;

    /// Convergence threshold for normalized variance
    double convergenceThreshold = 0.01;

    /// Time to hold in steady state before terminating (seconds)
    float holdTime = 1.0f;
};

/// MSER-based steady state detection condition
/// Uses a simplified MSER (Marginal Standard Error Rule) algorithm
/// to detect when a metric has reached steady state
class MSERSteadyStateCondition : public ITerminationCondition
{
public:
    /// @param metric The metric to monitor for steady state
    /// @param config MSER configuration
    explicit MSERSteadyStateCondition(
        MetricPtr metric,
        const MSERConfig& config = MSERConfig());

    std::string getName() const override { return "mser_steady_state"; }
    std::string getDescription() const override;

    bool shouldTerminate(physx::PxScene* scene, float time) const override;
    void reset() override;
    std::unique_ptr<ITerminationCondition> clone() const override;

    /// Get the detected truncation point (sample index where steady state begins)
    size_t getTruncationPoint() const { return m_truncationPoint; }

    /// Check if steady state has been detected
    bool hasReachedSteadyState() const { return m_reachedSteadyState; }

    /// Get the steady state mean value
    double getSteadyStateMean() const { return m_steadyStateMean; }

private:
    MetricPtr m_metric;
    MSERConfig m_config;

    mutable std::deque<double> m_samples;
    mutable size_t m_truncationPoint = 0;
    mutable bool m_reachedSteadyState = false;
    mutable double m_steadyStateMean = 0.0;
    mutable float m_timeBelowThreshold = 0.0f;
    mutable float m_lastTime = 0.0f;

    /// Compute MSER for a given truncation point
    double computeMSER(size_t truncationPoint) const;

    /// Find the optimal truncation point
    size_t findOptimalTruncationPoint() const;

    /// Check if converged based on MSER
    bool checkConvergence() const;
};

/// Factory function to create MSER condition for kinetic energy
inline std::unique_ptr<MSERSteadyStateCondition> createKineticEnergyMSERCondition(
    const MSERConfig& config = MSERConfig())
{
    // Create a custom metric that tracks kinetic energy
    auto metric = std::make_shared<CustomMetric>(
        "kinetic_energy_for_mser",
        [](physx::PxScene* scene, float time, float dt) -> MetricValue
        {
            (void)time;
            (void)dt;

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
                if (dynamic)
                {
                    float mass = dynamic->getMass();
                    physx::PxVec3 vel = dynamic->getLinearVelocity();
                    totalEnergy += 0.5f * mass * vel.magnitudeSquared();
                }
            }
            return totalEnergy;
        },
        "Kinetic energy tracked for MSER steady state detection"
    );

    return std::make_unique<MSERSteadyStateCondition>(metric, config);
}

} // namespace batch
