#pragma once

#include "Bond.h"
#include "../SceneLoader.h"
#include <PxPhysicsAPI.h>
#include <string>
#include <memory>

namespace bonding
{

/// Configuration for creating bonds
struct BondConfig
{
    /// Whether the bond can break under force
    bool breakable = false;

    /// Maximum force before break (if breakable)
    float breakForce = PX_MAX_F32;

    /// Maximum torque before break (if breakable)
    float breakTorque = PX_MAX_F32;

    /// Spring stiffness for compliant bonds
    float stiffness = 1000.0f;

    /// Damping coefficient for compliant bonds
    float damping = 10.0f;

    /// Rest length for distance-based bonds (0 = auto-calculate)
    float restLength = 0.0f;

    /// Minimum distance for distance joints
    float minDistance = 0.0f;

    /// Maximum distance for distance joints
    float maxDistance = 0.0f;

    /// Whether to enable collision between bonded actors
    bool enableCollision = false;
};

/// Interface for bond type implementations
/// Each bond type creates a different kind of PhysX joint
class IBondType
{
public:
    virtual ~IBondType() = default;

    /// Get the unique name of this bond type
    virtual std::string getName() const = 0;

    /// Get a description of this bond type
    virtual std::string getDescription() const = 0;

    /// Create a PhysX joint for this bond type
    /// @param physics PhysX physics object
    /// @param actor1 First actor to connect
    /// @param frame1 Local frame on first actor
    /// @param actor2 Second actor to connect
    /// @param frame2 Local frame on second actor
    /// @param bond Bond data (for additional configuration)
    /// @param config Bond configuration
    /// @return Created joint, or nullptr on failure
    virtual physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const = 0;

    /// Get a JointDef that represents this bond type
    /// Useful for serialization and scene export
    virtual JointDef getJointDef(const Bond& bond, const BondConfig& config) const = 0;

    /// Clone this bond type
    virtual std::unique_ptr<IBondType> clone() const = 0;
};

/// Shared pointer type for bond types
using BondTypePtr = std::shared_ptr<IBondType>;

} // namespace bonding
