#pragma once

#include "BondableEntityDef.h"
#include "Bond.h"
#include <PxPhysicsAPI.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>

namespace bonding
{

/// Runtime representation of a bondable entity
/// Tracks the entity's current bonding state and provides world-space queries
class BondableEntity
{
public:
    BondableEntity() = default;

    /// Initialize from definition
    void initialize(const BondableEntityDef& def, physx::PxRigidActor* actor);

    /// Get the entity's unique ID
    uint64_t getEntityId() const { return m_entityId; }

    /// Get the entity type
    const std::string& getEntityType() const { return m_entityType; }

    /// Get the PhysX actor
    physx::PxRigidActor* getActor() const { return m_actor; }

    /// Get the underlying definition
    const BondableEntityDef& getDefinition() const { return m_definition; }

    /// Get properties
    PropertyMap& getProperties() { return m_properties; }
    const PropertyMap& getProperties() const { return m_properties; }

    // --- World-space queries ---

    /// Get the world position of a bonding site
    physx::PxVec3 getSiteWorldPosition(uint32_t siteId) const;

    /// Get the world direction of a bonding site
    physx::PxVec3 getSiteWorldDirection(uint32_t siteId) const;

    /// Get the entity's current world transform
    physx::PxTransform getWorldTransform() const;

    // --- Bonding state queries ---

    /// Get all available (non-saturated) site IDs
    std::vector<uint32_t> getAvailableSites() const;

    /// Check if a specific site can accept another bond
    bool canBondAt(uint32_t siteId) const;

    /// Get the current number of bonds at a site
    uint32_t getBondCountAt(uint32_t siteId) const;

    /// Get the maximum valency of a site
    uint32_t getMaxValencyAt(uint32_t siteId) const;

    /// Get all bonds at a specific site
    std::vector<uint64_t> getBondsAt(uint32_t siteId) const;

    /// Get all bond IDs this entity participates in
    std::vector<uint64_t> getAllBondIds() const;

    /// Get total number of bonds across all sites
    size_t getTotalBondCount() const;

    /// Check if entity is fully saturated (all sites at max valency)
    bool isFullySaturated() const;

    // --- Bonding operations (called by DynamicBondManager) ---

    /// Record that a bond was formed at a site
    void recordBond(uint32_t siteId, uint64_t bondId);

    /// Remove a bond record from a site
    void removeBond(uint32_t siteId, uint64_t bondId);

    /// Clear all bond records (used when entity is removed)
    void clearAllBonds();

    // --- Site lookup ---

    /// Get the site definition
    const BondingSiteDef* getSiteDef(uint32_t siteId) const;

    /// Get all site definitions
    const std::vector<BondingSiteDef>& getAllSites() const;

    /// Get number of bonding sites
    size_t getSiteCount() const { return m_definition.bondingSites.size(); }

private:
    uint64_t m_entityId = 0;
    std::string m_entityType;
    physx::PxRigidActor* m_actor = nullptr;
    BondableEntityDef m_definition;
    PropertyMap m_properties;

    /// Map from site ID to set of bond IDs at that site
    std::unordered_map<uint32_t, std::unordered_set<uint64_t>> m_siteBonds;
};

} // namespace bonding
