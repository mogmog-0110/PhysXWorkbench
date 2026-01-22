#pragma once

#include <PxPhysicsAPI.h>
#include <string>
#include <memory>

namespace batch
{

/// Interface for simulation termination conditions
/// Conditions determine when a simulation run should end
class ITerminationCondition
{
public:
    virtual ~ITerminationCondition() = default;

    /// Get the unique name of this condition
    virtual std::string getName() const = 0;

    /// Get a description of what this condition checks
    virtual std::string getDescription() const { return ""; }

    /// Check if the simulation should terminate
    /// @param scene PhysX scene
    /// @param time Current simulation time
    /// @return true if simulation should terminate
    virtual bool shouldTerminate(physx::PxScene* scene, float time) const = 0;

    /// Reset the condition for a new simulation run
    virtual void reset() = 0;

    /// Clone this condition
    virtual std::unique_ptr<ITerminationCondition> clone() const = 0;
};

/// Shared pointer type for termination conditions
using TerminationConditionPtr = std::shared_ptr<ITerminationCondition>;

} // namespace batch
