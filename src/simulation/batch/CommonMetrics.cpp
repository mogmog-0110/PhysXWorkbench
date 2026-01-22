#include "CommonMetrics.h"
#include <algorithm>
#include <queue>
#include <unordered_map>

namespace batch
{

// =============================================================================
// BondCountMetric
// =============================================================================

BondCountMetric::BondCountMetric(bonding::DynamicBondManager* bondManager, bool trackTimeSeries)
    : m_bondManager(bondManager)
    , m_trackTimeSeries(trackTimeSeries)
{
}

void BondCountMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)scene;
    (void)dt;

    if (!m_bondManager)
        return;

    m_currentCount = static_cast<int>(m_bondManager->getBondCount());

    if (m_trackTimeSeries)
    {
        m_timeSeries.push_back({time, m_currentCount});
    }
}

MetricValue BondCountMetric::getValue() const
{
    return m_currentCount;
}

std::vector<std::pair<float, MetricValue>> BondCountMetric::getTimeSeries() const
{
    std::vector<std::pair<float, MetricValue>> result;
    result.reserve(m_timeSeries.size());
    for (const auto& [time, value] : m_timeSeries)
    {
        result.push_back({time, value});
    }
    return result;
}

void BondCountMetric::reset()
{
    m_currentCount = 0;
    m_timeSeries.clear();
}

std::unique_ptr<IMetric> BondCountMetric::clone() const
{
    return std::make_unique<BondCountMetric>(m_bondManager, m_trackTimeSeries);
}

// =============================================================================
// RingFormationMetric
// =============================================================================

RingFormationMetric::RingFormationMetric(bonding::DynamicBondManager* bondManager, int targetRingSize)
    : m_bondManager(bondManager)
    , m_targetRingSize(targetRingSize)
{
}

std::string RingFormationMetric::getDescription() const
{
    if (m_targetRingSize > 0)
        return "Detects ring formation of size " + std::to_string(m_targetRingSize);
    return "Detects any ring formation";
}

void RingFormationMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)scene;
    (void)dt;

    if (m_ringFormed)
        return; // Already found a ring

    int ringSize = 0;
    if (detectRing(ringSize))
    {
        m_ringFormed = true;
        m_ringFormationTime = time;
        m_detectedRingSize = ringSize;
    }
}

MetricValue RingFormationMetric::getValue() const
{
    if (m_ringFormed)
        return m_detectedRingSize;
    return 0;
}

void RingFormationMetric::reset()
{
    m_ringFormed = false;
    m_ringFormationTime = -1.0f;
    m_detectedRingSize = 0;
}

std::unique_ptr<IMetric> RingFormationMetric::clone() const
{
    return std::make_unique<RingFormationMetric>(m_bondManager, m_targetRingSize);
}

bool RingFormationMetric::detectRing(int& ringSize) const
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
            // Found a cycle
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
// KineticEnergyMetric
// =============================================================================

KineticEnergyMetric::KineticEnergyMetric(bool trackTimeSeries)
    : m_trackTimeSeries(trackTimeSeries)
{
}

void KineticEnergyMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)dt;

    if (!scene)
        return;

    float totalEnergy = 0.0f;

    // Get all dynamic actors
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
        physx::PxVec3 linearVel = dynamic->getLinearVelocity();
        physx::PxVec3 angularVel = dynamic->getAngularVelocity();

        // Linear kinetic energy: 0.5 * m * v^2
        float linearKE = 0.5f * mass * linearVel.magnitudeSquared();

        // Angular kinetic energy: 0.5 * I * omega^2 (simplified)
        physx::PxVec3 inertia = dynamic->getMassSpaceInertiaTensor();
        float angularKE = 0.5f * (inertia.x * angularVel.x * angularVel.x +
                                  inertia.y * angularVel.y * angularVel.y +
                                  inertia.z * angularVel.z * angularVel.z);

        totalEnergy += linearKE + angularKE;
    }

    m_currentEnergy = totalEnergy;

    if (m_trackTimeSeries)
    {
        m_timeSeries.push_back({time, totalEnergy});
    }
}

MetricValue KineticEnergyMetric::getValue() const
{
    return m_currentEnergy;
}

std::vector<std::pair<float, MetricValue>> KineticEnergyMetric::getTimeSeries() const
{
    std::vector<std::pair<float, MetricValue>> result;
    result.reserve(m_timeSeries.size());
    for (const auto& [time, value] : m_timeSeries)
    {
        result.push_back({time, value});
    }
    return result;
}

void KineticEnergyMetric::reset()
{
    m_currentEnergy = 0.0f;
    m_timeSeries.clear();
}

std::unique_ptr<IMetric> KineticEnergyMetric::clone() const
{
    return std::make_unique<KineticEnergyMetric>(m_trackTimeSeries);
}

// =============================================================================
// ClusterSizeMetric
// =============================================================================

ClusterSizeMetric::ClusterSizeMetric(bonding::DynamicBondManager* bondManager)
    : m_bondManager(bondManager)
{
}

void ClusterSizeMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)scene;
    (void)time;
    (void)dt;

    findClusters();
}

MetricValue ClusterSizeMetric::getValue() const
{
    return std::vector<int>(m_clusterSizes);
}

void ClusterSizeMetric::reset()
{
    m_clusterSizes.clear();
    m_largestClusterSize = 0;
}

std::unique_ptr<IMetric> ClusterSizeMetric::clone() const
{
    return std::make_unique<ClusterSizeMetric>(m_bondManager);
}

void ClusterSizeMetric::findClusters()
{
    m_clusterSizes.clear();
    m_largestClusterSize = 0;

    if (!m_bondManager)
        return;

    // Build adjacency list
    std::unordered_map<uint64_t, std::vector<uint64_t>> adjacency;
    auto entityIds = m_bondManager->getEntityIds();

    for (uint64_t id : entityIds)
    {
        adjacency[id] = {}; // Initialize even for isolated entities
    }

    const auto& bonds = m_bondManager->getBonds();
    for (const auto& [bondId, bond] : bonds)
    {
        adjacency[bond.endpoint1.entityId].push_back(bond.endpoint2.entityId);
        adjacency[bond.endpoint2.entityId].push_back(bond.endpoint1.entityId);
    }

    // BFS to find connected components
    std::unordered_set<uint64_t> visited;

    for (uint64_t startId : entityIds)
    {
        if (visited.count(startId))
            continue;

        int clusterSize = 0;
        std::queue<uint64_t> queue;
        queue.push(startId);
        visited.insert(startId);

        while (!queue.empty())
        {
            uint64_t current = queue.front();
            queue.pop();
            clusterSize++;

            for (uint64_t neighbor : adjacency[current])
            {
                if (!visited.count(neighbor))
                {
                    visited.insert(neighbor);
                    queue.push(neighbor);
                }
            }
        }

        m_clusterSizes.push_back(clusterSize);
        m_largestClusterSize = std::max(m_largestClusterSize, clusterSize);
    }

    // Sort cluster sizes descending
    std::sort(m_clusterSizes.begin(), m_clusterSizes.end(), std::greater<int>());
}

// =============================================================================
// DistanceMetric
// =============================================================================

DistanceMetric::DistanceMetric(
    uint64_t entity1Id, uint64_t entity2Id,
    bonding::DynamicBondManager* bondManager,
    bool trackTimeSeries)
    : m_entity1Id(entity1Id)
    , m_entity2Id(entity2Id)
    , m_bondManager(bondManager)
    , m_trackTimeSeries(trackTimeSeries)
{
}

std::string DistanceMetric::getDescription() const
{
    return "Distance between entities " + std::to_string(m_entity1Id) +
           " and " + std::to_string(m_entity2Id);
}

void DistanceMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)scene;
    (void)dt;

    if (!m_bondManager)
        return;

    const bonding::BondableEntity* e1 = m_bondManager->getEntity(m_entity1Id);
    const bonding::BondableEntity* e2 = m_bondManager->getEntity(m_entity2Id);

    if (!e1 || !e2)
    {
        m_currentDistance = -1.0f; // Invalid
        return;
    }

    physx::PxVec3 pos1 = e1->getWorldTransform().p;
    physx::PxVec3 pos2 = e2->getWorldTransform().p;
    m_currentDistance = (pos2 - pos1).magnitude();

    if (m_trackTimeSeries)
    {
        m_timeSeries.push_back({time, m_currentDistance});
    }
}

MetricValue DistanceMetric::getValue() const
{
    return m_currentDistance;
}

std::vector<std::pair<float, MetricValue>> DistanceMetric::getTimeSeries() const
{
    std::vector<std::pair<float, MetricValue>> result;
    result.reserve(m_timeSeries.size());
    for (const auto& [time, value] : m_timeSeries)
    {
        result.push_back({time, value});
    }
    return result;
}

void DistanceMetric::reset()
{
    m_currentDistance = 0.0f;
    m_timeSeries.clear();
}

std::unique_ptr<IMetric> DistanceMetric::clone() const
{
    return std::make_unique<DistanceMetric>(m_entity1Id, m_entity2Id, m_bondManager, m_trackTimeSeries);
}

// =============================================================================
// CustomMetric
// =============================================================================

CustomMetric::CustomMetric(
    const std::string& name,
    UpdateFunc updateFunc,
    const std::string& description)
    : m_name(name)
    , m_description(description)
    , m_updateFunc(std::move(updateFunc))
    , m_currentValue(0)
{
}

void CustomMetric::update(physx::PxScene* scene, float time, float dt)
{
    if (m_updateFunc)
    {
        m_currentValue = m_updateFunc(scene, time, dt);
    }
}

MetricValue CustomMetric::getValue() const
{
    return m_currentValue;
}

void CustomMetric::reset()
{
    m_currentValue = MetricValue(0);
}

std::unique_ptr<IMetric> CustomMetric::clone() const
{
    return std::make_unique<CustomMetric>(m_name, m_updateFunc, m_description);
}

// =============================================================================
// EntityCountMetric
// =============================================================================

EntityCountMetric::EntityCountMetric(bonding::DynamicBondManager* bondManager)
    : m_bondManager(bondManager)
{
}

void EntityCountMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)scene;
    (void)time;
    (void)dt;

    if (m_bondManager)
    {
        m_currentCount = static_cast<int>(m_bondManager->getEntityCount());
    }
}

MetricValue EntityCountMetric::getValue() const
{
    return m_currentCount;
}

void EntityCountMetric::reset()
{
    m_currentCount = 0;
}

std::unique_ptr<IMetric> EntityCountMetric::clone() const
{
    return std::make_unique<EntityCountMetric>(m_bondManager);
}

// =============================================================================
// AvailableSiteCountMetric
// =============================================================================

AvailableSiteCountMetric::AvailableSiteCountMetric(bonding::DynamicBondManager* bondManager)
    : m_bondManager(bondManager)
{
}

void AvailableSiteCountMetric::update(physx::PxScene* scene, float time, float dt)
{
    (void)scene;
    (void)time;
    (void)dt;

    if (m_bondManager)
    {
        auto stats = m_bondManager->getStats();
        m_currentCount = static_cast<int>(stats.availableSiteCount);
    }
}

MetricValue AvailableSiteCountMetric::getValue() const
{
    return m_currentCount;
}

void AvailableSiteCountMetric::reset()
{
    m_currentCount = 0;
}

std::unique_ptr<IMetric> AvailableSiteCountMetric::clone() const
{
    return std::make_unique<AvailableSiteCountMetric>(m_bondManager);
}

} // namespace batch
