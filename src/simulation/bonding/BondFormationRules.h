#pragma once

#include "IBondFormationRule.h"
#include <PxPhysicsAPI.h>

namespace bonding
{

/// Rule: Sites must be within a capture distance
/// Score decreases as distance increases (inverse relationship)
class ProximityRule : public IBondFormationRule
{
public:
    /// @param captureDistance Maximum distance for bond formation
    /// @param minDistance Minimum distance (bonds cannot form closer than this)
    explicit ProximityRule(float captureDistance = 2.0f, float minDistance = 0.0f);

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 100; } // High priority - check first
    std::string getName() const override { return "proximity"; }
    std::string getDescription() const override;

    float getCaptureDistance() const { return m_captureDistance; }
    void setCaptureDistance(float distance) { m_captureDistance = distance; }

private:
    float m_captureDistance;
    float m_minDistance;
};

/// Rule: Site directions must be properly aligned
/// For head-to-head bonding: directions should be anti-parallel
/// For head-to-tail bonding: directions should be parallel
class DirectionalAlignmentRule : public IBondFormationRule
{
public:
    enum class AlignmentMode
    {
        ANTIPARALLEL,  // Directions point toward each other (typical bond)
        PARALLEL,      // Directions point same way
        ANY            // No directional constraint
    };

    /// @param mode Alignment mode
    /// @param angleTolerance Maximum angle deviation in radians
    explicit DirectionalAlignmentRule(
        AlignmentMode mode = AlignmentMode::ANTIPARALLEL,
        float angleTolerance = 0.5236f); // ~30 degrees

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 80; }
    std::string getName() const override { return "directional_alignment"; }
    std::string getDescription() const override;

private:
    AlignmentMode m_mode;
    float m_angleTolerance;
};

/// Rule: Site types must be compatible
/// Uses the compatibleTypes list on BondingSiteDef
class TypeCompatibilityRule : public IBondFormationRule
{
public:
    TypeCompatibilityRule() = default;

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 90; } // Check after proximity
    std::string getName() const override { return "type_compatibility"; }
    std::string getDescription() const override;
};

/// Rule: Sites must have available valency
class ValencyRule : public IBondFormationRule
{
public:
    ValencyRule() = default;

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 95; } // Very high priority
    std::string getName() const override { return "valency"; }
    std::string getDescription() const override;
};

/// Rule: New bond must be coplanar with existing bonds
/// Used for ring formation to ensure planarity
class CoplanarityRule : public IBondFormationRule
{
public:
    /// @param maxDeviationAngle Maximum angle from the plane in radians
    explicit CoplanarityRule(float maxDeviationAngle = 0.087f); // ~5 degrees

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 50; }
    std::string getName() const override { return "coplanarity"; }
    std::string getDescription() const override;
    bool isHardConstraint() const override { return false; } // Scoring preference

private:
    float m_maxDeviationAngle;

    // Helper to compute plane normal from existing bonds
    bool computeBondPlane(
        const BondableEntity& entity,
        physx::PxVec3& planeNormal) const;
};

/// Rule: Bond angle with respect to existing bonds must be within range
/// E.g., for square rings, bond angles should be ~90 degrees
class AngleConstraintRule : public IBondFormationRule
{
public:
    /// @param targetAngle Target angle in radians
    /// @param tolerance Allowed deviation in radians
    explicit AngleConstraintRule(
        float targetAngle = physx::PxPi / 2.0f,  // 90 degrees
        float tolerance = 0.087f);               // ~5 degrees

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 60; }
    std::string getName() const override { return "angle_constraint"; }
    std::string getDescription() const override;

    float getTargetAngle() const { return m_targetAngle; }
    float getTolerance() const { return m_tolerance; }

private:
    float m_targetAngle;
    float m_tolerance;

    // Helper to get existing bond directions
    std::vector<physx::PxVec3> getExistingBondDirections(
        const BondableEntity& entity,
        uint32_t siteId) const;
};

/// Rule: Prevent self-bonding (entity bonding to itself)
class NoSelfBondingRule : public IBondFormationRule
{
public:
    NoSelfBondingRule() = default;

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 99; } // Very high priority
    std::string getName() const override { return "no_self_bonding"; }
    std::string getDescription() const override;
};

/// Rule: Prevent duplicate bonds (same entity pair already bonded)
class NoDuplicateBondRule : public IBondFormationRule
{
public:
    NoDuplicateBondRule() = default;

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override { return 98; }
    std::string getName() const override { return "no_duplicate_bond"; }
    std::string getDescription() const override;
};

/// Composite rule: Combines multiple rules with AND logic
/// All rules must pass (no negative scores) for bond to form
class CompositeRule : public IBondFormationRule
{
public:
    CompositeRule() = default;

    void addRule(BondFormationRulePtr rule);
    void removeRule(const std::string& ruleName);
    void clearRules();

    float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const override;

    int getPriority() const override;
    std::string getName() const override { return "composite"; }
    std::string getDescription() const override;

    const std::vector<BondFormationRulePtr>& getRules() const { return m_rules; }

private:
    std::vector<BondFormationRulePtr> m_rules;
};

} // namespace bonding
