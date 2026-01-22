#pragma once

#include "IMetric.h"
#include "ITerminationCondition.h"
#include "../bonding/DynamicBondManager.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>
#include <atomic>

namespace batch
{

/// Configuration for batch simulation runs
struct BatchConfig
{
    /// Number of replicate simulations to run
    int numReplicates = 10;

    /// Base seed for random number generation (each replicate uses baseSeed + replicateId)
    uint32_t baseSeed = 42;

    /// Physics timestep in seconds
    float timestep = 1.0f / 60.0f;

    /// Maximum simulation time per replicate (safety limit)
    float maxSimulationTime = 60.0f;

    /// Update interval for metrics (seconds, 0 = every frame)
    float metricUpdateInterval = 0.0f;

    /// Output directory for results
    std::string outputDirectory = "./results";

    /// Whether to run simulations in headless mode (no rendering)
    bool headless = true;

    /// Optional progress callback (called after each replicate)
    std::function<void(int completedReplicates, int totalReplicates)> progressCallback;
};

/// Result of a single simulation run
struct SimulationResult
{
    /// Replicate ID (0-based)
    int replicateId = 0;

    /// Random seed used for this replicate
    uint32_t seed = 0;

    /// Total simulation time when terminated
    float totalTime = 0.0f;

    /// Reason for termination (condition name)
    std::string terminationReason;

    /// Whether the simulation completed successfully
    bool success = true;

    /// Error message if success is false
    std::string errorMessage;

    /// Final metric values
    std::unordered_map<std::string, MetricValue> finalMetrics;

    /// Optional time series data for each metric
    std::unordered_map<std::string, std::vector<std::pair<float, MetricValue>>> timeSeries;
};

/// Factory function for creating scenes
/// @param physics PhysX physics object
/// @param scene PhysX scene to populate
/// @param bondManager Bond manager to populate
/// @param seed Random seed for this run
using SceneFactory = std::function<void(
    physx::PxPhysics* physics,
    physx::PxScene* scene,
    bonding::DynamicBondManager* bondManager,
    uint32_t seed)>;

/// Runs batch simulations and collects statistics
class BatchSimulationRunner
{
public:
    BatchSimulationRunner();
    ~BatchSimulationRunner();

    /// Configure the batch runner
    void configure(const BatchConfig& config);

    /// Get current configuration
    const BatchConfig& getConfig() const { return m_config; }

    /// Add a metric to track
    void addMetric(MetricPtr metric);

    /// Remove a metric by name
    void removeMetric(const std::string& name);

    /// Clear all metrics
    void clearMetrics();

    /// Add a termination condition
    void addTerminationCondition(TerminationConditionPtr condition);

    /// Remove a termination condition by name
    void removeTerminationCondition(const std::string& name);

    /// Clear all termination conditions
    void clearTerminationConditions();

    /// Run all simulations
    /// @param physics PhysX physics object to use
    /// @param sceneFactory Factory function to create scenes
    /// @return Results from all replicates
    std::vector<SimulationResult> run(
        physx::PxPhysics* physics,
        SceneFactory sceneFactory);

    /// Run a single replicate (for testing/debugging)
    SimulationResult runSingleReplicate(
        physx::PxPhysics* physics,
        SceneFactory sceneFactory,
        int replicateId,
        uint32_t seed);

    /// Cancel running batch (call from another thread)
    void cancel();

    /// Check if batch is currently running
    bool isRunning() const { return m_running.load(); }

    /// Export results to CSV file
    /// @param results Simulation results
    /// @param filename Output filename
    /// @return true on success
    static bool exportToCSV(
        const std::vector<SimulationResult>& results,
        const std::string& filename);

    /// Export results to JSON file
    /// @param results Simulation results
    /// @param filename Output filename
    /// @return true on success
    static bool exportToJSON(
        const std::vector<SimulationResult>& results,
        const std::string& filename);

    /// Calculate summary statistics from results
    struct SummaryStats
    {
        int totalReplicates = 0;
        int successfulReplicates = 0;
        float meanTime = 0.0f;
        float stdTime = 0.0f;
        float minTime = 0.0f;
        float maxTime = 0.0f;
        std::unordered_map<std::string, int> terminationReasonCounts;
        std::unordered_map<std::string, float> meanMetricValues;
    };

    static SummaryStats calculateSummary(const std::vector<SimulationResult>& results);

private:
    BatchConfig m_config;
    std::vector<MetricPtr> m_metrics;
    std::vector<TerminationConditionPtr> m_conditions;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_cancelled{false};

    /// Create a fresh PhysX scene for a replicate
    physx::PxScene* createScene(physx::PxPhysics* physics);

    /// Run the simulation loop for one replicate
    void runSimulationLoop(
        physx::PxScene* scene,
        bonding::DynamicBondManager* bondManager,
        SimulationResult& result);

    /// Check all termination conditions
    std::string checkTermination(physx::PxScene* scene, float time);

    /// Update all metrics
    void updateMetrics(physx::PxScene* scene, float time, float dt);

    /// Collect final metric values
    void collectFinalMetrics(SimulationResult& result);
};

} // namespace batch
