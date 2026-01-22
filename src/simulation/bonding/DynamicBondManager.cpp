#include "DynamicBondManager.h"
#include "BondFormationRules.h"
#include "BondTypes.h"
#include <algorithm>
#include <cmath>

namespace bonding
{

DynamicBondManager::DynamicBondManager()
{
    // Register default bond types
    m_bondTypes["rigid"] = std::make_shared<RigidBondType>();
    m_bondTypes["compliant"] = std::make_shared<CompliantBondType>();
    m_bondTypes["hinged"] = std::make_shared<HingedBondType>();
    m_bondTypes["ball_socket"] = std::make_shared<BallSocketBondType>();
    m_bondTypes["prismatic"] = std::make_shared<PrismaticBondType>();
    m_bondTypes["d6"] = std::make_shared<D6BondType>();

    // Add default rules
    addRule(std::make_shared<NoSelfBondingRule>());
    addRule(std::make_shared<NoDuplicateBondRule>());
    addRule(std::make_shared<ValencyRule>());
    addRule(std::make_shared<ProximityRule>(m_config.captureDistance));
    addRule(std::make_shared<TypeCompatibilityRule>());
}

DynamicBondManager::~DynamicBondManager()
{
    releaseAll();
}

void DynamicBondManager::initialize(physx::PxPhysics* physics, physx::PxScene* scene)
{
    m_physics = physics;
    m_scene = scene;
}

void DynamicBondManager::configure(const DynamicBondManagerConfig& config)
{
    m_config = config;

    // Update proximity rule if it exists
    for (auto& rule : m_rules)
    {
        if (auto* proximityRule = dynamic_cast<ProximityRule*>(rule.get()))
        {
            proximityRule->setCaptureDistance(config.captureDistance);
            break;
        }
    }
}

// =============================================================================
// Entity Management
// =============================================================================

uint64_t DynamicBondManager::registerEntity(const BondableEntityDef& def, physx::PxRigidActor* existingActor)
{
    if (!m_physics || !m_scene)
        return 0;

    // Get or create actor
    physx::PxRigidActor* actor = existingActor;
    if (!actor)
    {
        actor = createActor(def.actorDef);
        if (!actor)
            return 0;

        m_scene->addActor(*actor);
    }

    // Assign entity ID
    uint64_t entityId = m_nextEntityId.fetch_add(1);

    // Create entity
    auto entity = std::make_unique<BondableEntity>();
    BondableEntityDef mutableDef = def;
    mutableDef.entityId = entityId;
    entity->initialize(mutableDef, actor);

    m_entities[entityId] = std::move(entity);

    return entityId;
}

void DynamicBondManager::unregisterEntity(uint64_t entityId)
{
    auto it = m_entities.find(entityId);
    if (it == m_entities.end())
        return;

    // Break all bonds involving this entity
    std::vector<uint64_t> bondsToBreak;
    for (const auto& [bondId, bond] : m_bonds)
    {
        if (bond.involvesEntity(entityId))
        {
            bondsToBreak.push_back(bondId);
        }
    }

    for (uint64_t bondId : bondsToBreak)
    {
        breakBond(bondId);
    }

    // Remove actor from scene
    physx::PxRigidActor* actor = it->second->getActor();
    if (actor && m_scene)
    {
        m_scene->removeActor(*actor);
        actor->release();
    }

    m_entities.erase(it);
}

BondableEntity* DynamicBondManager::getEntity(uint64_t entityId)
{
    auto it = m_entities.find(entityId);
    return (it != m_entities.end()) ? it->second.get() : nullptr;
}

const BondableEntity* DynamicBondManager::getEntity(uint64_t entityId) const
{
    auto it = m_entities.find(entityId);
    return (it != m_entities.end()) ? it->second.get() : nullptr;
}

std::vector<uint64_t> DynamicBondManager::getEntityIds() const
{
    std::vector<uint64_t> ids;
    ids.reserve(m_entities.size());
    for (const auto& [id, entity] : m_entities)
    {
        ids.push_back(id);
    }
    return ids;
}

// =============================================================================
// Rule Management
// =============================================================================

void DynamicBondManager::addRule(BondFormationRulePtr rule)
{
    m_rules.push_back(std::move(rule));

    // Sort by priority (descending)
    std::sort(m_rules.begin(), m_rules.end(),
        [](const BondFormationRulePtr& a, const BondFormationRulePtr& b)
        {
            return a->getPriority() > b->getPriority();
        });
}

void DynamicBondManager::removeRule(const std::string& ruleName)
{
    m_rules.erase(
        std::remove_if(m_rules.begin(), m_rules.end(),
            [&ruleName](const BondFormationRulePtr& rule)
            {
                return rule->getName() == ruleName;
            }),
        m_rules.end());
}

void DynamicBondManager::clearRules()
{
    m_rules.clear();
}

// =============================================================================
// Bond Type Management
// =============================================================================

void DynamicBondManager::registerBondType(const std::string& name, BondTypePtr bondType)
{
    m_bondTypes[name] = std::move(bondType);
}

IBondType* DynamicBondManager::getBondType(const std::string& name)
{
    auto it = m_bondTypes.find(name);
    return (it != m_bondTypes.end()) ? it->second.get() : nullptr;
}

const IBondType* DynamicBondManager::getBondType(const std::string& name) const
{
    auto it = m_bondTypes.find(name);
    return (it != m_bondTypes.end()) ? it->second.get() : nullptr;
}

void DynamicBondManager::setDefaultBondType(const std::string& name)
{
    if (m_bondTypes.find(name) != m_bondTypes.end())
    {
        m_config.defaultBondType = name;
    }
}

// =============================================================================
// Bond Operations
// =============================================================================

uint64_t DynamicBondManager::createBond(
    uint64_t entity1Id, uint32_t site1Id,
    uint64_t entity2Id, uint32_t site2Id,
    const std::string& bondTypeName,
    const BondConfig* config)
{
    BondableEntity* e1 = getEntity(entity1Id);
    BondableEntity* e2 = getEntity(entity2Id);

    if (!e1 || !e2)
        return 0;

    // Use provided config or default
    BondConfig bondConfig = config ? *config : m_config.defaultBondConfig;

    // Use provided bond type or default
    std::string typeName = bondTypeName.empty() ? m_config.defaultBondType : bondTypeName;

    return createBondInternal(*e1, site1Id, *e2, site2Id, typeName, bondConfig);
}

void DynamicBondManager::breakBond(uint64_t bondId)
{
    auto it = m_bonds.find(bondId);
    if (it == m_bonds.end())
        return;

    Bond& bond = it->second;

    // Release the PhysX joint
    if (bond.joint)
    {
        bond.joint->release();
        bond.joint = nullptr;
    }

    // Update entity bond records
    if (BondableEntity* e1 = getEntity(bond.endpoint1.entityId))
    {
        e1->removeBond(bond.endpoint1.siteId, bondId);
    }
    if (BondableEntity* e2 = getEntity(bond.endpoint2.entityId))
    {
        e2->removeBond(bond.endpoint2.siteId, bondId);
    }

    // Fire callback
    fireBondBroken(bond, false);

    // Remove from map
    m_bonds.erase(it);
    m_bondsBrokenThisFrame++;
}

const Bond* DynamicBondManager::getBond(uint64_t bondId) const
{
    auto it = m_bonds.find(bondId);
    return (it != m_bonds.end()) ? &it->second : nullptr;
}

std::vector<const Bond*> DynamicBondManager::getBondsForEntity(uint64_t entityId) const
{
    std::vector<const Bond*> result;
    for (const auto& [bondId, bond] : m_bonds)
    {
        if (bond.involvesEntity(entityId))
        {
            result.push_back(&bond);
        }
    }
    return result;
}

// =============================================================================
// Simulation Update
// =============================================================================

void DynamicBondManager::update(float dt)
{
    if (!m_physics || !m_scene)
        return;

    m_simulationTime += dt;
    m_timeSinceLastCheck += dt;
    m_bondsFormedThisFrame = 0;
    m_bondsBrokenThisFrame = 0;

    // Check for broken bonds
    checkBrokenBonds();

    // Check for new bonds periodically
    if (m_timeSinceLastCheck >= m_config.proximityCheckInterval)
    {
        m_timeSinceLastCheck = 0.0f;

        // Update spatial hash
        if (m_config.enableSpatialHashing)
        {
            updateSpatialHash();
        }

        // Find and create bonds
        auto candidates = findBondCandidates();

        // Sort by score (descending)
        std::sort(candidates.begin(), candidates.end(),
            [](const BondCandidate& a, const BondCandidate& b)
            {
                return a.score > b.score;
            });

        // Create bonds (limited by maxBondsPerFrame)
        for (size_t i = 0; i < candidates.size() && m_bondsFormedThisFrame < m_config.maxBondsPerFrame; ++i)
        {
            const auto& candidate = candidates[i];

            BondableEntity* e1 = getEntity(candidate.entity1Id);
            BondableEntity* e2 = getEntity(candidate.entity2Id);

            if (!e1 || !e2)
                continue;

            // Verify sites are still available (might have been taken by earlier bonds)
            if (!e1->canBondAt(candidate.site1Id) || !e2->canBondAt(candidate.site2Id))
                continue;

            uint64_t bondId = createBondInternal(
                *e1, candidate.site1Id,
                *e2, candidate.site2Id,
                m_config.defaultBondType,
                m_config.defaultBondConfig);

            if (bondId != 0)
            {
                m_bondsFormedThisFrame++;
            }
        }
    }
}

// =============================================================================
// Callbacks
// =============================================================================

void DynamicBondManager::onBondFormed(BondFormedCallback callback)
{
    m_bondFormedCallbacks.push_back(std::move(callback));
}

void DynamicBondManager::onBondBroken(BondBrokenCallback callback)
{
    m_bondBrokenCallbacks.push_back(std::move(callback));
}

void DynamicBondManager::clearCallbacks()
{
    m_bondFormedCallbacks.clear();
    m_bondBrokenCallbacks.clear();
}

// =============================================================================
// Statistics
// =============================================================================

BondManagerStats DynamicBondManager::getStats() const
{
    BondManagerStats stats;
    stats.entityCount = m_entities.size();
    stats.bondCount = m_bonds.size();
    stats.bondsFormedThisFrame = m_bondsFormedThisFrame;
    stats.bondsBrokenThisFrame = m_bondsBrokenThisFrame;
    stats.lastUpdateTime = m_simulationTime;

    size_t availableSites = 0;
    size_t saturatedEntities = 0;

    for (const auto& [id, entity] : m_entities)
    {
        auto available = entity->getAvailableSites();
        availableSites += available.size();
        if (entity->isFullySaturated())
        {
            saturatedEntities++;
        }
    }

    stats.availableSiteCount = availableSites;
    stats.saturatedEntityCount = saturatedEntities;

    return stats;
}

// =============================================================================
// Cleanup
// =============================================================================

void DynamicBondManager::releaseAllBonds()
{
    // Release all joints
    for (auto& [bondId, bond] : m_bonds)
    {
        if (bond.joint)
        {
            bond.joint->release();
            bond.joint = nullptr;
        }
    }
    m_bonds.clear();

    // Clear bond records from entities
    for (auto& [entityId, entity] : m_entities)
    {
        entity->clearAllBonds();
    }
}

void DynamicBondManager::releaseAll()
{
    releaseAllBonds();

    // Release all actors
    for (auto& [entityId, entity] : m_entities)
    {
        physx::PxRigidActor* actor = entity->getActor();
        if (actor && m_scene)
        {
            m_scene->removeActor(*actor);
            actor->release();
        }
    }
    m_entities.clear();

    m_simulationTime = 0.0f;
    m_timeSinceLastCheck = 0.0f;
    m_spatialHash.clear();
}

// =============================================================================
// Internal Methods
// =============================================================================

physx::PxRigidActor* DynamicBondManager::createActor(const ActorDef& def)
{
    if (!m_physics)
        return nullptr;

    // Create transform
    physx::PxTransform transform(
        physx::PxVec3(def.posX, def.posY, def.posZ),
        physx::PxQuat(def.quatX, def.quatY, def.quatZ, def.quatW));

    // Create material
    physx::PxMaterial* material = m_physics->createMaterial(
        def.staticFriction, def.dynamicFriction, def.restitution);

    if (!material)
        return nullptr;

    // Create actor based on type
    physx::PxRigidActor* actor = nullptr;

    if (def.type == ActorDef::Type::STATIC)
    {
        actor = m_physics->createRigidStatic(transform);
    }
    else
    {
        physx::PxRigidDynamic* dynamic = m_physics->createRigidDynamic(transform);
        if (dynamic)
        {
            // Set initial velocity
            dynamic->setLinearVelocity(physx::PxVec3(def.velX, def.velY, def.velZ));
            dynamic->setAngularVelocity(physx::PxVec3(def.angVelX, def.angVelY, def.angVelZ));
        }
        actor = dynamic;
    }

    if (!actor)
    {
        material->release();
        return nullptr;
    }

    // Create shape based on geometry type
    physx::PxShape* shape = nullptr;

    switch (def.geomType)
    {
    case ActorDef::GeomType::SPHERE:
        shape = m_physics->createShape(physx::PxSphereGeometry(def.sphereRadius), *material);
        break;

    case ActorDef::GeomType::BOX:
        shape = m_physics->createShape(
            physx::PxBoxGeometry(def.boxHalfX, def.boxHalfY, def.boxHalfZ), *material);
        break;

    case ActorDef::GeomType::CAPSULE:
        shape = m_physics->createShape(
            physx::PxCapsuleGeometry(def.capsuleRadius, def.capsuleHalfHeight), *material);
        break;

    default:
        // Default to sphere
        shape = m_physics->createShape(physx::PxSphereGeometry(0.5f), *material);
        break;
    }

    if (shape)
    {
        actor->attachShape(*shape);
        shape->release(); // Actor holds reference

        // Update mass for dynamic actors
        if (def.type == ActorDef::Type::DYNAMIC)
        {
            physx::PxRigidBodyExt::updateMassAndInertia(
                *static_cast<physx::PxRigidDynamic*>(actor), def.density);
        }
    }

    material->release(); // Shape holds reference

    return actor;
}

void DynamicBondManager::checkBrokenBonds()
{
    std::vector<uint64_t> brokenBonds;

    for (auto& [bondId, bond] : m_bonds)
    {
        if (!bond.isValid())
        {
            brokenBonds.push_back(bondId);
        }
    }

    for (uint64_t bondId : brokenBonds)
    {
        auto it = m_bonds.find(bondId);
        if (it != m_bonds.end())
        {
            Bond& bond = it->second;

            // Update entity records
            if (BondableEntity* e1 = getEntity(bond.endpoint1.entityId))
            {
                e1->removeBond(bond.endpoint1.siteId, bondId);
            }
            if (BondableEntity* e2 = getEntity(bond.endpoint2.entityId))
            {
                e2->removeBond(bond.endpoint2.siteId, bondId);
            }

            // Fire callback (broken by force)
            fireBondBroken(bond, true);

            // Release joint if still valid
            if (bond.joint)
            {
                bond.joint->release();
            }

            m_bonds.erase(it);
            m_bondsBrokenThisFrame++;
        }
    }
}

void DynamicBondManager::updateSpatialHash()
{
    m_spatialHash.clear();

    for (const auto& [entityId, entity] : m_entities)
    {
        for (const auto& site : entity->getAllSites())
        {
            if (!entity->canBondAt(site.siteId))
                continue;

            physx::PxVec3 pos = entity->getSiteWorldPosition(site.siteId);
            int64_t key = getSpatialKey(pos);
            m_spatialHash[key].sites.push_back({entityId, site.siteId});
        }
    }
}

int64_t DynamicBondManager::getSpatialKey(const physx::PxVec3& pos) const
{
    int32_t x = static_cast<int32_t>(std::floor(pos.x / m_config.spatialCellSize));
    int32_t y = static_cast<int32_t>(std::floor(pos.y / m_config.spatialCellSize));
    int32_t z = static_cast<int32_t>(std::floor(pos.z / m_config.spatialCellSize));

    // Pack into 64-bit key (21 bits per dimension + sign)
    return (static_cast<int64_t>(x) & 0x1FFFFF) |
           ((static_cast<int64_t>(y) & 0x1FFFFF) << 21) |
           ((static_cast<int64_t>(z) & 0x1FFFFF) << 42);
}

std::vector<int64_t> DynamicBondManager::getNeighborCells(const physx::PxVec3& pos) const
{
    std::vector<int64_t> neighbors;
    neighbors.reserve(27);

    float cellSize = m_config.spatialCellSize;
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dz = -1; dz <= 1; ++dz)
            {
                physx::PxVec3 neighborPos(
                    pos.x + dx * cellSize,
                    pos.y + dy * cellSize,
                    pos.z + dz * cellSize);
                neighbors.push_back(getSpatialKey(neighborPos));
            }
        }
    }

    return neighbors;
}

std::vector<DynamicBondManager::BondCandidate> DynamicBondManager::findBondCandidates()
{
    std::vector<BondCandidate> candidates;

    if (m_config.enableSpatialHashing)
    {
        // Use spatial hash for efficient lookup
        for (const auto& [cellKey, cell] : m_spatialHash)
        {
            // Get neighboring cells
            physx::PxVec3 cellCenter; // Approximate center
            auto neighborKeys = getNeighborCells(cellCenter);

            // Check pairs within this cell and neighbors
            for (size_t i = 0; i < cell.sites.size(); ++i)
            {
                const auto& [entityId1, siteId1] = cell.sites[i];
                const BondableEntity* e1 = getEntity(entityId1);
                if (!e1)
                    continue;

                // Same cell pairs
                for (size_t j = i + 1; j < cell.sites.size(); ++j)
                {
                    const auto& [entityId2, siteId2] = cell.sites[j];
                    const BondableEntity* e2 = getEntity(entityId2);
                    if (!e2)
                        continue;

                    float score = evaluateRules(*e1, siteId1, *e2, siteId2);
                    if (score > 0)
                    {
                        candidates.push_back({entityId1, entityId2, siteId1, siteId2, score});
                    }
                }

                // Neighbor cell pairs
                for (int64_t neighborKey : neighborKeys)
                {
                    if (neighborKey <= cellKey)
                        continue; // Avoid duplicates

                    auto neighborIt = m_spatialHash.find(neighborKey);
                    if (neighborIt == m_spatialHash.end())
                        continue;

                    for (const auto& [entityId2, siteId2] : neighborIt->second.sites)
                    {
                        const BondableEntity* e2 = getEntity(entityId2);
                        if (!e2)
                            continue;

                        float score = evaluateRules(*e1, siteId1, *e2, siteId2);
                        if (score > 0)
                        {
                            candidates.push_back({entityId1, entityId2, siteId1, siteId2, score});
                        }
                    }
                }
            }
        }
    }
    else
    {
        // Brute force all pairs
        std::vector<uint64_t> entityIds;
        for (const auto& [id, entity] : m_entities)
        {
            entityIds.push_back(id);
        }

        for (size_t i = 0; i < entityIds.size(); ++i)
        {
            const BondableEntity* e1 = getEntity(entityIds[i]);
            if (!e1)
                continue;

            auto sites1 = e1->getAvailableSites();
            if (sites1.empty())
                continue;

            for (size_t j = i + 1; j < entityIds.size(); ++j)
            {
                const BondableEntity* e2 = getEntity(entityIds[j]);
                if (!e2)
                    continue;

                auto sites2 = e2->getAvailableSites();
                if (sites2.empty())
                    continue;

                // Check all site pairs
                for (uint32_t s1 : sites1)
                {
                    for (uint32_t s2 : sites2)
                    {
                        float score = evaluateRules(*e1, s1, *e2, s2);
                        if (score > 0)
                        {
                            candidates.push_back({entityIds[i], entityIds[j], s1, s2, score});
                        }
                    }
                }
            }
        }
    }

    return candidates;
}

float DynamicBondManager::evaluateRules(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    float combinedScore = 1.0f;

    for (const auto& rule : m_rules)
    {
        float score = rule->evaluate(e1, s1, e2, s2);

        if (score < 0.0f)
            return -1.0f; // Veto

        if (score > 0.0f)
            combinedScore *= score;
    }

    return combinedScore;
}

uint64_t DynamicBondManager::createBondInternal(
    BondableEntity& e1, uint32_t s1,
    BondableEntity& e2, uint32_t s2,
    const std::string& bondTypeName,
    const BondConfig& config)
{
    // Get bond type
    IBondType* bondType = getBondType(bondTypeName);
    if (!bondType)
    {
        bondType = getBondType(m_config.defaultBondType);
        if (!bondType)
            return 0;
    }

    // Get actors
    physx::PxRigidActor* actor1 = e1.getActor();
    physx::PxRigidActor* actor2 = e2.getActor();

    if (!actor1 || !actor2)
        return 0;

    // Compute joint frames
    const BondingSiteDef* site1 = e1.getSiteDef(s1);
    const BondingSiteDef* site2 = e2.getSiteDef(s2);

    if (!site1 || !site2)
        return 0;

    physx::PxTransform frame1(site1->getLocalPosition());
    physx::PxTransform frame2(site2->getLocalPosition());

    // Create bond ID
    uint64_t bondId = m_nextBondId.fetch_add(1);

    // Create bond structure
    Bond bond;
    bond.bondId = bondId;
    bond.endpoint1 = {e1.getEntityId(), s1};
    bond.endpoint2 = {e2.getEntityId(), s2};
    bond.bondType = bondTypeName;
    bond.formationTime = m_simulationTime;

    // Create PhysX joint
    bond.joint = bondType->createJoint(m_physics, actor1, frame1, actor2, frame2, bond, config);

    if (!bond.joint)
        return 0;

    // Record bond in entities
    e1.recordBond(s1, bondId);
    e2.recordBond(s2, bondId);

    // Store bond
    m_bonds[bondId] = bond;

    // Fire callback
    fireBondFormed(bond);

    return bondId;
}

void DynamicBondManager::fireBondFormed(const Bond& bond)
{
    BondFormedEvent event{bond, m_simulationTime};
    for (const auto& callback : m_bondFormedCallbacks)
    {
        callback(event);
    }
}

void DynamicBondManager::fireBondBroken(const Bond& bond, bool wasBreakForce)
{
    BondBrokenEvent event{bond, m_simulationTime, wasBreakForce};
    for (const auto& callback : m_bondBrokenCallbacks)
    {
        callback(event);
    }
}

} // namespace bonding
