#pragma once

#include "PropertyMap.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>
#include <cstdint>

namespace bonding
{

/// Definition of a bonding site on an entity
/// A bonding site is a point on an entity where bonds can form
struct BondingSiteDef
{
    /// Unique site identifier within the entity
    uint32_t siteId = 0;

    /// Local position relative to entity's body frame
    float localPosX = 0.0f;
    float localPosY = 0.0f;
    float localPosZ = 0.0f;

    /// Local direction for directional bonding (normalized)
    /// Bonds prefer to form along this direction
    float localDirX = 1.0f;
    float localDirY = 0.0f;
    float localDirZ = 0.0f;

    /// Maximum number of bonds that can form at this site
    uint32_t maxValency = 1;

    /// Site type identifier for compatibility checking
    std::string siteType = "default";

    /// List of site types this site can bond with
    /// Empty list means compatible with any type
    std::vector<std::string> compatibleTypes;

    /// Custom properties for domain-specific data
    PropertyMap properties;

    /// Helper: Get local position as PxVec3
    physx::PxVec3 getLocalPosition() const
    {
        return physx::PxVec3(localPosX, localPosY, localPosZ);
    }

    /// Helper: Set local position from PxVec3
    void setLocalPosition(const physx::PxVec3& pos)
    {
        localPosX = pos.x;
        localPosY = pos.y;
        localPosZ = pos.z;
    }

    /// Helper: Get local direction as PxVec3
    physx::PxVec3 getLocalDirection() const
    {
        return physx::PxVec3(localDirX, localDirY, localDirZ);
    }

    /// Helper: Set local direction from PxVec3 (will be normalized)
    void setLocalDirection(const physx::PxVec3& dir)
    {
        physx::PxVec3 normalized = dir.getNormalized();
        localDirX = normalized.x;
        localDirY = normalized.y;
        localDirZ = normalized.z;
    }

    /// Check if this site is compatible with another site type
    bool isCompatibleWith(const std::string& otherType) const
    {
        // Empty compatible list means compatible with anything
        if (compatibleTypes.empty())
            return true;

        for (const auto& type : compatibleTypes)
        {
            if (type == otherType)
                return true;
        }
        return false;
    }
};

} // namespace bonding
