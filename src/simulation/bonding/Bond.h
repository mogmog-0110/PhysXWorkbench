#pragma once

#include <PxPhysicsAPI.h>
#include <string>
#include <cstdint>

namespace bonding
{

/// Represents one endpoint of a bond
struct BondEndpoint
{
    uint64_t entityId = 0;   ///< Entity this endpoint belongs to
    uint32_t siteId = 0;     ///< Bonding site on the entity

    bool operator==(const BondEndpoint& other) const
    {
        return entityId == other.entityId && siteId == other.siteId;
    }

    bool operator!=(const BondEndpoint& other) const
    {
        return !(*this == other);
    }
};

/// Represents a bond between two entities
struct Bond
{
    /// Unique bond identifier
    uint64_t bondId = 0;

    /// The two endpoints of the bond
    BondEndpoint endpoint1;
    BondEndpoint endpoint2;

    /// The PhysX joint implementing this bond (owned by PhysX)
    physx::PxJoint* joint = nullptr;

    /// Bond type identifier (e.g., "rigid", "compliant", "hinged")
    std::string bondType = "rigid";

    /// Simulation time when the bond was formed
    float formationTime = 0.0f;

    /// Whether the bond has been marked for removal
    bool pendingRemoval = false;

    /// Check if this bond connects a specific entity
    bool involvesEntity(uint64_t entityId) const
    {
        return endpoint1.entityId == entityId || endpoint2.entityId == entityId;
    }

    /// Check if this bond connects a specific site
    bool involvesSite(uint64_t entityId, uint32_t siteId) const
    {
        return (endpoint1.entityId == entityId && endpoint1.siteId == siteId) ||
               (endpoint2.entityId == entityId && endpoint2.siteId == siteId);
    }

    /// Get the other endpoint given one entity ID
    const BondEndpoint* getOtherEndpoint(uint64_t entityId) const
    {
        if (endpoint1.entityId == entityId)
            return &endpoint2;
        if (endpoint2.entityId == entityId)
            return &endpoint1;
        return nullptr;
    }

    /// Check if the joint is still valid (not broken)
    bool isValid() const
    {
        if (!joint || pendingRemoval)
            return false;

        // Check if joint still has valid actors
        physx::PxRigidActor* actor0 = nullptr;
        physx::PxRigidActor* actor1 = nullptr;
        joint->getActors(actor0, actor1);

        return actor0 != nullptr && actor1 != nullptr;
    }
};

/// Event data for bond formation
struct BondFormedEvent
{
    const Bond& bond;
    float simulationTime;
};

/// Event data for bond breaking
struct BondBrokenEvent
{
    Bond bond;  // Copy since bond may be deleted
    float simulationTime;
    bool wasBreakForce;  // True if broken by force, false if manually removed
};

} // namespace bonding
