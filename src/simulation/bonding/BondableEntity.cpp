#include "BondableEntity.h"

namespace bonding
{

void BondableEntity::initialize(const BondableEntityDef& def, physx::PxRigidActor* actor)
{
    m_definition = def;
    m_entityId = def.entityId;
    m_entityType = def.entityType;
    m_actor = actor;
    m_properties = def.properties;

    // Initialize site bond tracking
    m_siteBonds.clear();
    for (const auto& site : m_definition.bondingSites)
    {
        m_siteBonds[site.siteId] = std::unordered_set<uint64_t>();
    }
}

physx::PxVec3 BondableEntity::getSiteWorldPosition(uint32_t siteId) const
{
    const BondingSiteDef* site = getSiteDef(siteId);
    if (!site || !m_actor)
        return physx::PxVec3(0.0f);

    physx::PxTransform worldTransform = m_actor->getGlobalPose();
    physx::PxVec3 localPos = site->getLocalPosition();

    return worldTransform.transform(localPos);
}

physx::PxVec3 BondableEntity::getSiteWorldDirection(uint32_t siteId) const
{
    const BondingSiteDef* site = getSiteDef(siteId);
    if (!site || !m_actor)
        return physx::PxVec3(1.0f, 0.0f, 0.0f);

    physx::PxTransform worldTransform = m_actor->getGlobalPose();
    physx::PxVec3 localDir = site->getLocalDirection();

    // Rotate direction without translation
    return worldTransform.q.rotate(localDir);
}

physx::PxTransform BondableEntity::getWorldTransform() const
{
    if (!m_actor)
        return physx::PxTransform(physx::PxIdentity);

    return m_actor->getGlobalPose();
}

std::vector<uint32_t> BondableEntity::getAvailableSites() const
{
    std::vector<uint32_t> available;

    for (const auto& site : m_definition.bondingSites)
    {
        if (canBondAt(site.siteId))
        {
            available.push_back(site.siteId);
        }
    }

    return available;
}

bool BondableEntity::canBondAt(uint32_t siteId) const
{
    const BondingSiteDef* site = getSiteDef(siteId);
    if (!site)
        return false;

    uint32_t currentBonds = getBondCountAt(siteId);
    return currentBonds < site->maxValency;
}

uint32_t BondableEntity::getBondCountAt(uint32_t siteId) const
{
    auto it = m_siteBonds.find(siteId);
    if (it == m_siteBonds.end())
        return 0;

    return static_cast<uint32_t>(it->second.size());
}

uint32_t BondableEntity::getMaxValencyAt(uint32_t siteId) const
{
    const BondingSiteDef* site = getSiteDef(siteId);
    if (!site)
        return 0;

    return site->maxValency;
}

std::vector<uint64_t> BondableEntity::getBondsAt(uint32_t siteId) const
{
    auto it = m_siteBonds.find(siteId);
    if (it == m_siteBonds.end())
        return {};

    return std::vector<uint64_t>(it->second.begin(), it->second.end());
}

std::vector<uint64_t> BondableEntity::getAllBondIds() const
{
    std::unordered_set<uint64_t> allBonds;

    for (const auto& [siteId, bonds] : m_siteBonds)
    {
        allBonds.insert(bonds.begin(), bonds.end());
    }

    return std::vector<uint64_t>(allBonds.begin(), allBonds.end());
}

size_t BondableEntity::getTotalBondCount() const
{
    std::unordered_set<uint64_t> allBonds;

    for (const auto& [siteId, bonds] : m_siteBonds)
    {
        allBonds.insert(bonds.begin(), bonds.end());
    }

    return allBonds.size();
}

bool BondableEntity::isFullySaturated() const
{
    for (const auto& site : m_definition.bondingSites)
    {
        if (canBondAt(site.siteId))
            return false;
    }
    return true;
}

void BondableEntity::recordBond(uint32_t siteId, uint64_t bondId)
{
    auto it = m_siteBonds.find(siteId);
    if (it == m_siteBonds.end())
    {
        // Site doesn't exist, create it
        m_siteBonds[siteId] = std::unordered_set<uint64_t>();
        it = m_siteBonds.find(siteId);
    }

    it->second.insert(bondId);
}

void BondableEntity::removeBond(uint32_t siteId, uint64_t bondId)
{
    auto it = m_siteBonds.find(siteId);
    if (it != m_siteBonds.end())
    {
        it->second.erase(bondId);
    }
}

void BondableEntity::clearAllBonds()
{
    for (auto& [siteId, bonds] : m_siteBonds)
    {
        bonds.clear();
    }
}

const BondingSiteDef* BondableEntity::getSiteDef(uint32_t siteId) const
{
    return m_definition.findSite(siteId);
}

const std::vector<BondingSiteDef>& BondableEntity::getAllSites() const
{
    return m_definition.bondingSites;
}

} // namespace bonding
