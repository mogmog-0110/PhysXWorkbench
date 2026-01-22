#pragma once

#include "IBondType.h"
#include <PxPhysicsAPI.h>

namespace bonding
{

/// Rigid (fixed) bond type
/// Creates a PxFixedJoint that locks both actors together
class RigidBondType : public IBondType
{
public:
    RigidBondType() = default;

    std::string getName() const override { return "rigid"; }
    std::string getDescription() const override;

    physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const override;

    JointDef getJointDef(const Bond& bond, const BondConfig& config) const override;
    std::unique_ptr<IBondType> clone() const override;
};

/// Compliant (spring-damper) bond type
/// Creates a PxDistanceJoint with spring behavior
class CompliantBondType : public IBondType
{
public:
    /// @param defaultStiffness Default spring stiffness (overridden by config)
    /// @param defaultDamping Default damping (overridden by config)
    explicit CompliantBondType(
        float defaultStiffness = 1000.0f,
        float defaultDamping = 10.0f);

    std::string getName() const override { return "compliant"; }
    std::string getDescription() const override;

    physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const override;

    JointDef getJointDef(const Bond& bond, const BondConfig& config) const override;
    std::unique_ptr<IBondType> clone() const override;

    float getDefaultStiffness() const { return m_defaultStiffness; }
    float getDefaultDamping() const { return m_defaultDamping; }

private:
    float m_defaultStiffness;
    float m_defaultDamping;
};

/// Hinged (revolute) bond type
/// Creates a PxRevoluteJoint allowing rotation around one axis
class HingedBondType : public IBondType
{
public:
    /// @param axis Local axis of rotation (normalized)
    /// @param enableLimits Whether to enable rotation limits
    /// @param lowerLimit Lower rotation limit in radians
    /// @param upperLimit Upper rotation limit in radians
    explicit HingedBondType(
        const physx::PxVec3& axis = physx::PxVec3(0, 0, 1),
        bool enableLimits = false,
        float lowerLimit = -physx::PxPi,
        float upperLimit = physx::PxPi);

    std::string getName() const override { return "hinged"; }
    std::string getDescription() const override;

    physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const override;

    JointDef getJointDef(const Bond& bond, const BondConfig& config) const override;
    std::unique_ptr<IBondType> clone() const override;

private:
    physx::PxVec3 m_axis;
    bool m_enableLimits;
    float m_lowerLimit;
    float m_upperLimit;

    // Helper to create rotation that aligns Z axis with desired axis
    physx::PxQuat computeAxisRotation() const;
};

/// Ball-socket (spherical) bond type
/// Creates a PxSphericalJoint allowing rotation around all axes
class BallSocketBondType : public IBondType
{
public:
    /// @param enableConeLimit Whether to enable cone limit
    /// @param coneAngle Maximum cone angle in radians (if limited)
    explicit BallSocketBondType(
        bool enableConeLimit = false,
        float coneAngle = physx::PxPi / 4.0f);

    std::string getName() const override { return "ball_socket"; }
    std::string getDescription() const override;

    physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const override;

    JointDef getJointDef(const Bond& bond, const BondConfig& config) const override;
    std::unique_ptr<IBondType> clone() const override;

private:
    bool m_enableConeLimit;
    float m_coneAngle;
};

/// Prismatic (sliding) bond type
/// Creates a PxPrismaticJoint allowing linear motion along one axis
class PrismaticBondType : public IBondType
{
public:
    /// @param axis Sliding axis (normalized)
    /// @param enableLimits Whether to enable position limits
    /// @param lowerLimit Lower position limit
    /// @param upperLimit Upper position limit
    explicit PrismaticBondType(
        const physx::PxVec3& axis = physx::PxVec3(1, 0, 0),
        bool enableLimits = false,
        float lowerLimit = -1.0f,
        float upperLimit = 1.0f);

    std::string getName() const override { return "prismatic"; }
    std::string getDescription() const override;

    physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const override;

    JointDef getJointDef(const Bond& bond, const BondConfig& config) const override;
    std::unique_ptr<IBondType> clone() const override;

private:
    physx::PxVec3 m_axis;
    bool m_enableLimits;
    float m_lowerLimit;
    float m_upperLimit;

    physx::PxQuat computeAxisRotation() const;
};

/// D6 (fully configurable) bond type
/// Creates a PxD6Joint with configurable DOF
class D6BondType : public IBondType
{
public:
    struct AxisConfig
    {
        physx::PxD6Motion::Enum motion = physx::PxD6Motion::eLOCKED;
        float limitLower = 0.0f;
        float limitUpper = 0.0f;
        float driveStiffness = 0.0f;
        float driveDamping = 0.0f;
    };

    D6BondType();

    std::string getName() const override { return "d6"; }
    std::string getDescription() const override;

    physx::PxJoint* createJoint(
        physx::PxPhysics* physics,
        physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
        physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
        const Bond& bond,
        const BondConfig& config) const override;

    JointDef getJointDef(const Bond& bond, const BondConfig& config) const override;
    std::unique_ptr<IBondType> clone() const override;

    // Configure individual axes
    void setLinearX(const AxisConfig& config) { m_linearX = config; }
    void setLinearY(const AxisConfig& config) { m_linearY = config; }
    void setLinearZ(const AxisConfig& config) { m_linearZ = config; }
    void setTwist(const AxisConfig& config) { m_twist = config; }
    void setSwing1(const AxisConfig& config) { m_swing1 = config; }
    void setSwing2(const AxisConfig& config) { m_swing2 = config; }

private:
    AxisConfig m_linearX, m_linearY, m_linearZ;
    AxisConfig m_twist, m_swing1, m_swing2;
};

} // namespace bonding
