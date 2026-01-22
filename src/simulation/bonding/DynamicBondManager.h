#pragma once

#include "BondableEntity.h"
#include "BondableEntityDef.h"
#include "Bond.h"
#include "IBondFormationRule.h"
#include "IBondType.h"
#include <PxPhysicsAPI.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <functional>
#include <atomic>
#include <cstdint>

namespace bonding
{

/// Configuration for the dynamic bond manager
struct DynamicBondManagerConfig
{
    /// How often to check for new bond formation (seconds)
    float proximityCheckInterval = 0.1f;

    /// Default capture distance for proximity checks
    float captureDistance = 2.0f;

    /// Maximum number of bonds to form per frame (for performance)
    uint32_t maxBondsPerFrame = 10;

    /// Enable spatial hashing for faster proximity checks
    bool enableSpatialHashing = true;

    /// Cell size for spatial hashing (should be >= captureDistance)
    float spatialCellSize = 5.0f;

    /// Default bond configuration
    BondConfig defaultBondConfig;

    /// Default bond type name
    std::string defaultBondType = "rigid";
};

/// Statistics about the bond manager state
struct BondManagerStats
{
    size_t entityCount = 0;
    size_t bondCount = 0;
    size_t availableSiteCount = 0;
    size_t saturatedEntityCount = 0;
    size_t bondsFormedThisFrame = 0;
    size_t bondsBrokenThisFrame = 0;
    float lastUpdateTime = 0.0f;
};

/// Callback types
using BondFormedCallback = std::function<void(const BondFormedEvent&)>;
using BondBrokenCallback = std::function<void(const BondBrokenEvent&)>;

/// Manages dynamic bond formation and breaking in a simulation
/// Central component that coordinates entities, bonds, rules, and bond types
class DynamicBondManager
{
public:
    DynamicBondManager();
    ~DynamicBondManager();

    // Disable copy
    DynamicBondManager(const DynamicBondManager&) = delete;
    DynamicBondManager& operator=(const DynamicBondManager&) = delete;

    /// Initialize the manager with PhysX references
    void initialize(physx::PxPhysics* physics, physx::PxScene* scene);

    /// Configure the manager
    void configure(const DynamicBondManagerConfig& config);

    /// Get the current configuration
    const DynamicBondManagerConfig& getConfig() const { return m_config; }

    // --- Entity Management ---

    /// Register a new bondable entity
    /// @param def Entity definition
    /// @param existingActor Optional existing actor (if nullptr, creates new)
    /// @return Assigned entity ID
    uint64_t registerEntity(const BondableEntityDef& def, physx::PxRigidActor* existingActor = nullptr);

    /// Unregister an entity and break all its bonds
    void unregisterEntity(uint64_t entityId);

    /// Get an entity by ID
    BondableEntity* getEntity(uint64_t entityId);
    const BondableEntity* getEntity(uint64_t entityId) const;

    /// Get all entity IDs
    std::vector<uint64_t> getEntityIds() const;

    /// Get entity count
    size_t getEntityCount() const { return m_entities.size(); }

    // --- Rule Management ---

    /// Add a bond formation rule
    void addRule(BondFormationRulePtr rule);

    /// Remove a rule by name
    void removeRule(const std::string& ruleName);

    /// Clear all rules
    void clearRules();

    /// Get all rules
    const std::vector<BondFormationRulePtr>& getRules() const { return m_rules; }

    // --- Bond Type Management ---

    /// Register a bond type
    void registerBondType(const std::string& name, BondTypePtr bondType);

    /// Get a bond type by name
    IBondType* getBondType(const std::string& name);
    const IBondType* getBondType(const std::string& name) const;

    /// Set default bond type
    void setDefaultBondType(const std::string& name);

    // --- Bond Operations ---

    /// Manually create a bond between two sites
    /// @return Bond ID, or 0 on failure
    uint64_t createBond(
        uint64_t entity1Id, uint32_t site1Id,
        uint64_t entity2Id, uint32_t site2Id,
        const std::string& bondTypeName = "",
        const BondConfig* config = nullptr);

    /// Manually break a bond
    void breakBond(uint64_t bondId);

    /// Get all bonds
    const std::unordered_map<uint64_t, Bond>& getBonds() const { return m_bonds; }

    /// Get bond count
    size_t getBondCount() const { return m_bonds.size(); }

    /// Get a bond by ID
    const Bond* getBond(uint64_t bondId) const;

    /// Get bonds involving a specific entity
    std::vector<const Bond*> getBondsForEntity(uint64_t entityId) const;

    // --- Simulation Update ---

    /// Update the bond manager (call every frame)
    /// @param dt Time step in seconds
    void update(float dt);

    /// Get current simulation time
    float getSimulationTime() const { return m_simulationTime; }

    /// Reset simulation time
    void resetSimulationTime() { m_simulationTime = 0.0f; }

    // --- Callbacks ---

    /// Register a callback for bond formation
    void onBondFormed(BondFormedCallback callback);

    /// Register a callback for bond breaking
    void onBondBroken(BondBrokenCallback callback);

    /// Clear all callbacks
    void clearCallbacks();

    // --- Statistics ---

    /// Get current statistics
    BondManagerStats getStats() const;

    // --- Cleanup ---

    /// Release all bonds (keep entities)
    void releaseAllBonds();

    /// Release everything
    void releaseAll();

private:
    // PhysX references
    physx::PxPhysics* m_physics = nullptr;
    physx::PxScene* m_scene = nullptr;

    // Configuration
    DynamicBondManagerConfig m_config;

    // Entity management
    std::unordered_map<uint64_t, std::unique_ptr<BondableEntity>> m_entities;
    std::atomic<uint64_t> m_nextEntityId{1};

    // Bond management
    std::unordered_map<uint64_t, Bond> m_bonds;
    std::atomic<uint64_t> m_nextBondId{1};

    // Rules and bond types
    std::vector<BondFormationRulePtr> m_rules;
    std::unordered_map<std::string, BondTypePtr> m_bondTypes;

    // Callbacks
    std::vector<BondFormedCallback> m_bondFormedCallbacks;
    std::vector<BondBrokenCallback> m_bondBrokenCallbacks;

    // Simulation state
    float m_simulationTime = 0.0f;
    float m_timeSinceLastCheck = 0.0f;

    // Per-frame statistics
    size_t m_bondsFormedThisFrame = 0;
    size_t m_bondsBrokenThisFrame = 0;

    // Spatial hashing
    struct SpatialCell
    {
        std::vector<std::pair<uint64_t, uint32_t>> sites; // entityId, siteId
    };
    std::unordered_map<int64_t, SpatialCell> m_spatialHash;

    // --- Internal methods ---

    /// Create PhysX actor from definition
    physx::PxRigidActor* createActor(const ActorDef& def);

    /// Check for broken joints and remove them
    void checkBrokenBonds();

    /// Update spatial hash with current site positions
    void updateSpatialHash();

    /// Get spatial hash key from position
    int64_t getSpatialKey(const physx::PxVec3& pos) const;

    /// Get neighboring cells for a position
    std::vector<int64_t> getNeighborCells(const physx::PxVec3& pos) const;

    /// Find potential bond candidates
    struct BondCandidate
    {
        uint64_t entity1Id, entity2Id;
        uint32_t site1Id, site2Id;
        float score;
    };
    std::vector<BondCandidate> findBondCandidates();

    /// Evaluate all rules for a site pair
    float evaluateRules(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const;

    /// Actually create the bond (internal)
    uint64_t createBondInternal(
        BondableEntity& e1, uint32_t s1,
        BondableEntity& e2, uint32_t s2,
        const std::string& bondTypeName,
        const BondConfig& config);

    /// Fire bond formed callbacks
    void fireBondFormed(const Bond& bond);

    /// Fire bond broken callbacks
    void fireBondBroken(const Bond& bond, bool wasBreakForce);
};

} // namespace bonding
