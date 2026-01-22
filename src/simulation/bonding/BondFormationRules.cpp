#include "BondFormationRules.h"
#include <algorithm>
#include <cmath>

namespace bonding
{

// =============================================================================
// ProximityRule
// =============================================================================

ProximityRule::ProximityRule(float captureDistance, float minDistance)
    : m_captureDistance(captureDistance)
    , m_minDistance(minDistance)
{
}

float ProximityRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    physx::PxVec3 pos1 = e1.getSiteWorldPosition(s1);
    physx::PxVec3 pos2 = e2.getSiteWorldPosition(s2);

    float distance = (pos2 - pos1).magnitude();

    // Too far - veto
    if (distance > m_captureDistance)
        return -1.0f;

    // Too close - veto
    if (distance < m_minDistance)
        return -1.0f;

    // Score: closer = higher score (linear falloff)
    float normalizedDistance = distance / m_captureDistance;
    return 1.0f - normalizedDistance;
}

std::string ProximityRule::getDescription() const
{
    return "Sites must be within " + std::to_string(m_captureDistance) + " distance units";
}

// =============================================================================
// DirectionalAlignmentRule
// =============================================================================

DirectionalAlignmentRule::DirectionalAlignmentRule(AlignmentMode mode, float angleTolerance)
    : m_mode(mode)
    , m_angleTolerance(angleTolerance)
{
}

float DirectionalAlignmentRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    if (m_mode == AlignmentMode::ANY)
        return 1.0f; // No constraint

    physx::PxVec3 dir1 = e1.getSiteWorldDirection(s1);
    physx::PxVec3 dir2 = e2.getSiteWorldDirection(s2);

    float dot = dir1.dot(dir2);

    float targetDot;
    switch (m_mode)
    {
    case AlignmentMode::ANTIPARALLEL:
        // Directions should point toward each other (dot = -1)
        targetDot = -1.0f;
        break;
    case AlignmentMode::PARALLEL:
        // Directions should point same way (dot = 1)
        targetDot = 1.0f;
        break;
    default:
        return 1.0f;
    }

    // Calculate angle deviation
    float angleDiff = std::acos(std::clamp(dot * targetDot, -1.0f, 1.0f));

    if (angleDiff > m_angleTolerance)
        return -1.0f; // Veto

    // Score: better alignment = higher score
    return 1.0f - (angleDiff / m_angleTolerance);
}

std::string DirectionalAlignmentRule::getDescription() const
{
    std::string modeStr;
    switch (m_mode)
    {
    case AlignmentMode::ANTIPARALLEL:
        modeStr = "anti-parallel";
        break;
    case AlignmentMode::PARALLEL:
        modeStr = "parallel";
        break;
    default:
        modeStr = "any";
        break;
    }
    return "Site directions must be " + modeStr + " within " +
           std::to_string(m_angleTolerance * 180.0f / physx::PxPi) + " degrees";
}

// =============================================================================
// TypeCompatibilityRule
// =============================================================================

float TypeCompatibilityRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    const BondingSiteDef* site1 = e1.getSiteDef(s1);
    const BondingSiteDef* site2 = e2.getSiteDef(s2);

    if (!site1 || !site2)
        return -1.0f;

    // Check bidirectional compatibility
    bool compatible = site1->isCompatibleWith(site2->siteType) &&
                      site2->isCompatibleWith(site1->siteType);

    return compatible ? 1.0f : -1.0f;
}

std::string TypeCompatibilityRule::getDescription() const
{
    return "Site types must be mutually compatible";
}

// =============================================================================
// ValencyRule
// =============================================================================

float ValencyRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    if (!e1.canBondAt(s1))
        return -1.0f;

    if (!e2.canBondAt(s2))
        return -1.0f;

    return 1.0f;
}

std::string ValencyRule::getDescription() const
{
    return "Both sites must have available valency";
}

// =============================================================================
// CoplanarityRule
// =============================================================================

CoplanarityRule::CoplanarityRule(float maxDeviationAngle)
    : m_maxDeviationAngle(maxDeviationAngle)
{
}

float CoplanarityRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    physx::PxVec3 planeNormal1, planeNormal2;
    bool hasPlane1 = computeBondPlane(e1, planeNormal1);
    bool hasPlane2 = computeBondPlane(e2, planeNormal2);

    // If neither entity has enough bonds to define a plane, allow the bond
    if (!hasPlane1 && !hasPlane2)
        return 1.0f;

    // Compute the direction of the proposed bond
    physx::PxVec3 pos1 = e1.getSiteWorldPosition(s1);
    physx::PxVec3 pos2 = e2.getSiteWorldPosition(s2);
    physx::PxVec3 bondDir = (pos2 - pos1).getNormalized();

    float maxDeviation = 0.0f;

    // Check deviation from plane 1
    if (hasPlane1)
    {
        float deviation = std::abs(bondDir.dot(planeNormal1));
        float angle = std::asin(std::clamp(deviation, 0.0f, 1.0f));
        maxDeviation = std::max(maxDeviation, angle);
    }

    // Check deviation from plane 2
    if (hasPlane2)
    {
        float deviation = std::abs(bondDir.dot(planeNormal2));
        float angle = std::asin(std::clamp(deviation, 0.0f, 1.0f));
        maxDeviation = std::max(maxDeviation, angle);
    }

    if (maxDeviation > m_maxDeviationAngle)
        return -1.0f; // Veto

    // Score: less deviation = higher score
    return 1.0f - (maxDeviation / m_maxDeviationAngle);
}

bool CoplanarityRule::computeBondPlane(
    const BondableEntity& entity,
    physx::PxVec3& planeNormal) const
{
    // Need at least 2 bonds to define a plane
    if (entity.getTotalBondCount() < 2)
        return false;

    // Get all bond directions
    std::vector<physx::PxVec3> directions;
    physx::PxVec3 entityPos = entity.getWorldTransform().p;

    for (const auto& site : entity.getAllSites())
    {
        auto bonds = entity.getBondsAt(site.siteId);
        for (auto bondId : bonds)
        {
            physx::PxVec3 sitePos = entity.getSiteWorldPosition(site.siteId);
            physx::PxVec3 dir = (sitePos - entityPos).getNormalized();
            if (dir.magnitude() > 0.001f)
            {
                directions.push_back(dir);
            }
        }
    }

    if (directions.size() < 2)
        return false;

    // Compute plane normal from first two bond directions
    planeNormal = directions[0].cross(directions[1]).getNormalized();
    return planeNormal.magnitude() > 0.001f;
}

std::string CoplanarityRule::getDescription() const
{
    return "Bonds must be coplanar within " +
           std::to_string(m_maxDeviationAngle * 180.0f / physx::PxPi) + " degrees";
}

// =============================================================================
// AngleConstraintRule
// =============================================================================

AngleConstraintRule::AngleConstraintRule(float targetAngle, float tolerance)
    : m_targetAngle(targetAngle)
    , m_tolerance(tolerance)
{
}

float AngleConstraintRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    // Get proposed bond direction
    physx::PxVec3 pos1 = e1.getSiteWorldPosition(s1);
    physx::PxVec3 pos2 = e2.getSiteWorldPosition(s2);
    physx::PxVec3 proposedDir = (pos2 - pos1).getNormalized();

    // Check against existing bonds from entity 1
    auto existingDirs1 = getExistingBondDirections(e1, s1);
    for (const auto& existingDir : existingDirs1)
    {
        float dot = proposedDir.dot(existingDir);
        float angle = std::acos(std::clamp(dot, -1.0f, 1.0f));
        float diff = std::abs(angle - m_targetAngle);

        if (diff > m_tolerance)
            return -1.0f;
    }

    // Check against existing bonds from entity 2
    physx::PxVec3 reversedDir = -proposedDir;
    auto existingDirs2 = getExistingBondDirections(e2, s2);
    for (const auto& existingDir : existingDirs2)
    {
        float dot = reversedDir.dot(existingDir);
        float angle = std::acos(std::clamp(dot, -1.0f, 1.0f));
        float diff = std::abs(angle - m_targetAngle);

        if (diff > m_tolerance)
            return -1.0f;
    }

    // If no existing bonds, allow formation
    if (existingDirs1.empty() && existingDirs2.empty())
        return 1.0f;

    // Score based on how close to target angle
    float score = 1.0f;
    if (!existingDirs1.empty())
    {
        float bestDiff = m_tolerance;
        for (const auto& existingDir : existingDirs1)
        {
            float dot = proposedDir.dot(existingDir);
            float angle = std::acos(std::clamp(dot, -1.0f, 1.0f));
            bestDiff = std::min(bestDiff, std::abs(angle - m_targetAngle));
        }
        score *= 1.0f - (bestDiff / m_tolerance);
    }

    return score;
}

std::vector<physx::PxVec3> AngleConstraintRule::getExistingBondDirections(
    const BondableEntity& entity,
    uint32_t siteId) const
{
    std::vector<physx::PxVec3> directions;
    physx::PxVec3 sitePos = entity.getSiteWorldPosition(siteId);

    // Get directions to all bonded sites on this entity
    for (const auto& site : entity.getAllSites())
    {
        if (site.siteId == siteId)
            continue;

        auto bonds = entity.getBondsAt(site.siteId);
        if (!bonds.empty())
        {
            physx::PxVec3 otherSitePos = entity.getSiteWorldPosition(site.siteId);
            physx::PxVec3 dir = (otherSitePos - sitePos).getNormalized();
            if (dir.magnitude() > 0.001f)
            {
                directions.push_back(dir);
            }
        }
    }

    return directions;
}

std::string AngleConstraintRule::getDescription() const
{
    return "Bond angle must be " + std::to_string(m_targetAngle * 180.0f / physx::PxPi) +
           " degrees +/- " + std::to_string(m_tolerance * 180.0f / physx::PxPi);
}

// =============================================================================
// NoSelfBondingRule
// =============================================================================

float NoSelfBondingRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    (void)s1; // Unused
    (void)s2; // Unused

    if (e1.getEntityId() == e2.getEntityId())
        return -1.0f;

    return 1.0f;
}

std::string NoSelfBondingRule::getDescription() const
{
    return "Entity cannot bond to itself";
}

// =============================================================================
// NoDuplicateBondRule
// =============================================================================

float NoDuplicateBondRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    // Check if there's already a bond between these exact sites
    auto bonds1 = e1.getBondsAt(s1);
    auto bonds2 = e2.getBondsAt(s2);

    // Find common bond IDs
    for (auto bondId1 : bonds1)
    {
        for (auto bondId2 : bonds2)
        {
            if (bondId1 == bondId2)
                return -1.0f; // Already bonded at these sites
        }
    }

    return 1.0f;
}

std::string NoDuplicateBondRule::getDescription() const
{
    return "Cannot form duplicate bond at same sites";
}

// =============================================================================
// CompositeRule
// =============================================================================

void CompositeRule::addRule(BondFormationRulePtr rule)
{
    m_rules.push_back(std::move(rule));

    // Sort by priority (descending)
    std::sort(m_rules.begin(), m_rules.end(),
        [](const BondFormationRulePtr& a, const BondFormationRulePtr& b)
        {
            return a->getPriority() > b->getPriority();
        });
}

void CompositeRule::removeRule(const std::string& ruleName)
{
    m_rules.erase(
        std::remove_if(m_rules.begin(), m_rules.end(),
            [&ruleName](const BondFormationRulePtr& rule)
            {
                return rule->getName() == ruleName;
            }),
        m_rules.end());
}

void CompositeRule::clearRules()
{
    m_rules.clear();
}

float CompositeRule::evaluate(
    const BondableEntity& e1, uint32_t s1,
    const BondableEntity& e2, uint32_t s2) const
{
    if (m_rules.empty())
        return 1.0f;

    float combinedScore = 1.0f;

    for (const auto& rule : m_rules)
    {
        float score = rule->evaluate(e1, s1, e2, s2);

        // Any negative score vetoes the bond
        if (score < 0.0f)
            return -1.0f;

        // Multiply positive scores
        if (score > 0.0f)
            combinedScore *= score;
    }

    return combinedScore;
}

int CompositeRule::getPriority() const
{
    if (m_rules.empty())
        return 0;

    // Return highest priority among contained rules
    int maxPriority = 0;
    for (const auto& rule : m_rules)
    {
        maxPriority = std::max(maxPriority, rule->getPriority());
    }
    return maxPriority;
}

std::string CompositeRule::getDescription() const
{
    std::string desc = "Composite rule with " + std::to_string(m_rules.size()) + " sub-rules: ";
    for (size_t i = 0; i < m_rules.size(); ++i)
    {
        if (i > 0)
            desc += ", ";
        desc += m_rules[i]->getName();
    }
    return desc;
}

} // namespace bonding
