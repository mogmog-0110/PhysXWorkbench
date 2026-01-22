#pragma once

#include "BondingSiteDef.h"
#include "PropertyMap.h"
#include "../SceneLoader.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <vector>
#include <cstdint>

namespace bonding
{

/// Definition for creating a bondable entity
/// Extends the basic ActorDef with bonding capabilities
struct BondableEntityDef
{
    /// The underlying physics actor definition
    ActorDef actorDef;

    /// Bonding sites on this entity
    std::vector<BondingSiteDef> bondingSites;

    /// Entity type identifier for classification
    std::string entityType = "particle";

    /// Unique entity ID (assigned by DynamicBondManager)
    uint64_t entityId = 0;

    /// Custom properties for domain-specific data
    PropertyMap properties;

    /// Default constructor
    BondableEntityDef()
    {
        // Initialize ActorDef defaults
        actorDef.type = ActorDef::Type::DYNAMIC;
        actorDef.geomType = ActorDef::GeomType::SPHERE;
        actorDef.posX = actorDef.posY = actorDef.posZ = 0.0f;
        actorDef.quatW = 1.0f;
        actorDef.quatX = actorDef.quatY = actorDef.quatZ = 0.0f;
        actorDef.sphereRadius = 0.5f;
        actorDef.boxHalfX = actorDef.boxHalfY = actorDef.boxHalfZ = 0.5f;
        actorDef.capsuleRadius = 0.25f;
        actorDef.capsuleHalfHeight = 0.5f;
        actorDef.density = 1.0f;
        actorDef.staticFriction = 0.5f;
        actorDef.dynamicFriction = 0.5f;
        actorDef.restitution = 0.6f;
        actorDef.velX = actorDef.velY = actorDef.velZ = 0.0f;
        actorDef.angVelX = actorDef.angVelY = actorDef.angVelZ = 0.0f;
    }

    /// Create a simple spherical entity with two bonding sites (dimer-style)
    static BondableEntityDef createDimer(float radius = 0.5f, float bondingSiteOffset = 0.0f)
    {
        BondableEntityDef def;
        def.actorDef.geomType = ActorDef::GeomType::SPHERE;
        def.actorDef.sphereRadius = radius;
        def.entityType = "dimer";

        float siteOffset = (bondingSiteOffset > 0.0f) ? bondingSiteOffset : radius;

        // Two bonding sites on opposite sides
        BondingSiteDef site1;
        site1.siteId = 0;
        site1.localPosX = siteOffset;
        site1.localPosY = 0.0f;
        site1.localPosZ = 0.0f;
        site1.localDirX = 1.0f;
        site1.localDirY = 0.0f;
        site1.localDirZ = 0.0f;
        site1.maxValency = 1;
        def.bondingSites.push_back(site1);

        BondingSiteDef site2;
        site2.siteId = 1;
        site2.localPosX = -siteOffset;
        site2.localPosY = 0.0f;
        site2.localPosZ = 0.0f;
        site2.localDirX = -1.0f;
        site2.localDirY = 0.0f;
        site2.localDirZ = 0.0f;
        site2.maxValency = 1;
        def.bondingSites.push_back(site2);

        return def;
    }

    /// Create a tetrahedral entity with four bonding sites
    static BondableEntityDef createTetrahedral(float radius = 0.5f)
    {
        BondableEntityDef def;
        def.actorDef.geomType = ActorDef::GeomType::SPHERE;
        def.actorDef.sphereRadius = radius;
        def.entityType = "tetrahedral";

        // Tetrahedral geometry (normalized directions)
        const float a = 1.0f / std::sqrt(3.0f);
        const float dirs[4][3] = {
            {  a,  a,  a },
            {  a, -a, -a },
            { -a,  a, -a },
            { -a, -a,  a }
        };

        for (uint32_t i = 0; i < 4; ++i)
        {
            BondingSiteDef site;
            site.siteId = i;
            site.localPosX = dirs[i][0] * radius;
            site.localPosY = dirs[i][1] * radius;
            site.localPosZ = dirs[i][2] * radius;
            site.localDirX = dirs[i][0];
            site.localDirY = dirs[i][1];
            site.localDirZ = dirs[i][2];
            site.maxValency = 1;
            def.bondingSites.push_back(site);
        }

        return def;
    }

    /// Get total bonding capacity (sum of all site valencies)
    uint32_t getTotalValency() const
    {
        uint32_t total = 0;
        for (const auto& site : bondingSites)
        {
            total += site.maxValency;
        }
        return total;
    }

    /// Find a bonding site by ID
    const BondingSiteDef* findSite(uint32_t siteId) const
    {
        for (const auto& site : bondingSites)
        {
            if (site.siteId == siteId)
                return &site;
        }
        return nullptr;
    }
};

} // namespace bonding
