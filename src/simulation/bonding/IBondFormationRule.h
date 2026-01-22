#pragma once

#include "BondableEntity.h"
#include <string>
#include <memory>

namespace bonding
{

/// Interface for bond formation rules
/// Rules evaluate whether a bond should form between two sites
/// and return a score indicating the quality of the potential bond.
///
/// Score interpretation:
///   < 0  : Bond cannot form (rule vetoes)
///   = 0  : Neutral (doesn't affect scoring)
///   > 0  : Bond can form, higher = better match
///
/// When multiple rules are combined:
///   - Any rule returning < 0 vetoes the bond
///   - Scores from positive rules are multiplied together
class IBondFormationRule
{
public:
    virtual ~IBondFormationRule() = default;

    /// Evaluate the potential bond between two sites
    /// @param e1 First entity
    /// @param s1 Site ID on first entity
    /// @param e2 Second entity
    /// @param s2 Site ID on second entity
    /// @return Score: negative = veto, 0 = neutral, positive = weighted score
    virtual float evaluate(
        const BondableEntity& e1, uint32_t s1,
        const BondableEntity& e2, uint32_t s2) const = 0;

    /// Get the priority of this rule (higher = evaluated first)
    /// Rules with higher priority can short-circuit evaluation
    virtual int getPriority() const { return 0; }

    /// Get the unique name of this rule
    virtual std::string getName() const = 0;

    /// Get a description of what this rule checks
    virtual std::string getDescription() const { return ""; }

    /// Whether this rule is a hard constraint (must pass) or soft (scoring only)
    virtual bool isHardConstraint() const { return true; }
};

/// Shared pointer type for rules
using BondFormationRulePtr = std::shared_ptr<IBondFormationRule>;

} // namespace bonding
