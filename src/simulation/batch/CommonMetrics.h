#pragma once

#include "IMetric.h"
#include "../bonding/DynamicBondManager.h"
#include <functional>
#include <unordered_set>

namespace batch
{

/// Metric: Tracks bond count over time
class BondCountMetric : public IMetric
{
public:
    /// @param bondManager Reference to the bond manager to track
    /// @param trackTimeSeries Whether to store the full time series
    explicit BondCountMetric(bonding::DynamicBondManager* bondManager, bool trackTimeSeries = false);

    std::string getName() const override { return "bond_count"; }
    std::string getDescription() const override { return "Number of bonds in the simulation"; }

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    std::vector<std::pair<float, MetricValue>> getTimeSeries() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

private:
    bonding::DynamicBondManager* m_bondManager;
    bool m_trackTimeSeries;
    int m_currentCount = 0;
    std::vector<std::pair<float, int>> m_timeSeries;
};

/// Metric: Detects ring formation (closed loops of bonds)
class RingFormationMetric : public IMetric
{
public:
    /// @param bondManager Reference to the bond manager
    /// @param targetRingSize Size of ring to detect (0 = any size)
    explicit RingFormationMetric(bonding::DynamicBondManager* bondManager, int targetRingSize = 0);

    std::string getName() const override { return "ring_formation"; }
    std::string getDescription() const override;

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

    /// Check if a ring has formed
    bool hasRingFormed() const { return m_ringFormed; }

    /// Get the time when ring was formed
    float getRingFormationTime() const { return m_ringFormationTime; }

    /// Get the size of detected ring
    int getRingSize() const { return m_detectedRingSize; }

private:
    bonding::DynamicBondManager* m_bondManager;
    int m_targetRingSize;
    bool m_ringFormed = false;
    float m_ringFormationTime = -1.0f;
    int m_detectedRingSize = 0;

    /// Detect ring using DFS
    bool detectRing(int& ringSize) const;
};

/// Metric: Tracks total kinetic energy
class KineticEnergyMetric : public IMetric
{
public:
    /// @param trackTimeSeries Whether to store the full time series
    explicit KineticEnergyMetric(bool trackTimeSeries = false);

    std::string getName() const override { return "kinetic_energy"; }
    std::string getDescription() const override { return "Total kinetic energy of all dynamic actors"; }

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    std::vector<std::pair<float, MetricValue>> getTimeSeries() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

private:
    bool m_trackTimeSeries;
    float m_currentEnergy = 0.0f;
    std::vector<std::pair<float, float>> m_timeSeries;
};

/// Metric: Tracks cluster sizes (connected components)
class ClusterSizeMetric : public IMetric
{
public:
    /// @param bondManager Reference to the bond manager
    explicit ClusterSizeMetric(bonding::DynamicBondManager* bondManager);

    std::string getName() const override { return "cluster_sizes"; }
    std::string getDescription() const override { return "Distribution of connected cluster sizes"; }

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

    /// Get the largest cluster size
    int getLargestClusterSize() const { return m_largestClusterSize; }

    /// Get the number of clusters
    int getClusterCount() const { return static_cast<int>(m_clusterSizes.size()); }

private:
    bonding::DynamicBondManager* m_bondManager;
    std::vector<int> m_clusterSizes;
    int m_largestClusterSize = 0;

    /// Find connected components
    void findClusters();
};

/// Metric: Tracks distance between specific entities
class DistanceMetric : public IMetric
{
public:
    /// @param entity1Id First entity ID
    /// @param entity2Id Second entity ID
    /// @param bondManager Reference to get entity positions
    /// @param trackTimeSeries Whether to store the full time series
    DistanceMetric(
        uint64_t entity1Id, uint64_t entity2Id,
        bonding::DynamicBondManager* bondManager,
        bool trackTimeSeries = false);

    std::string getName() const override { return "distance"; }
    std::string getDescription() const override;

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    std::vector<std::pair<float, MetricValue>> getTimeSeries() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

private:
    uint64_t m_entity1Id;
    uint64_t m_entity2Id;
    bonding::DynamicBondManager* m_bondManager;
    bool m_trackTimeSeries;
    float m_currentDistance = 0.0f;
    std::vector<std::pair<float, float>> m_timeSeries;
};

/// Metric: Custom metric using a lambda function
class CustomMetric : public IMetric
{
public:
    using UpdateFunc = std::function<MetricValue(physx::PxScene*, float, float)>;

    /// @param name Name of the metric
    /// @param updateFunc Function to compute the metric value
    /// @param description Optional description
    CustomMetric(
        const std::string& name,
        UpdateFunc updateFunc,
        const std::string& description = "");

    std::string getName() const override { return m_name; }
    std::string getDescription() const override { return m_description; }

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

private:
    std::string m_name;
    std::string m_description;
    UpdateFunc m_updateFunc;
    MetricValue m_currentValue;
};

/// Metric: Tracks entity count
class EntityCountMetric : public IMetric
{
public:
    explicit EntityCountMetric(bonding::DynamicBondManager* bondManager);

    std::string getName() const override { return "entity_count"; }
    std::string getDescription() const override { return "Number of bondable entities"; }

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

private:
    bonding::DynamicBondManager* m_bondManager;
    int m_currentCount = 0;
};

/// Metric: Tracks available (unsaturated) site count
class AvailableSiteCountMetric : public IMetric
{
public:
    explicit AvailableSiteCountMetric(bonding::DynamicBondManager* bondManager);

    std::string getName() const override { return "available_sites"; }
    std::string getDescription() const override { return "Number of available bonding sites"; }

    void update(physx::PxScene* scene, float time, float dt) override;
    MetricValue getValue() const override;
    void reset() override;
    std::unique_ptr<IMetric> clone() const override;

private:
    bonding::DynamicBondManager* m_bondManager;
    int m_currentCount = 0;
};

} // namespace batch
