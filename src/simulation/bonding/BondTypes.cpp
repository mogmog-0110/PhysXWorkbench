#include "BondTypes.h"
#include <cmath>
#include <algorithm>

// Helper function for clamping (C++17 compatible)
namespace {
    template<typename T>
    T clampValue(T value, T minVal, T maxVal) {
        return (value < minVal) ? minVal : ((value > maxVal) ? maxVal : value);
    }
}

namespace bonding
{

// =============================================================================
// RigidBondType
// =============================================================================

std::string RigidBondType::getDescription() const
{
    return "Fixed joint that locks both actors together (no relative motion)";
}

physx::PxJoint* RigidBondType::createJoint(
    physx::PxPhysics* physics,
    physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
    physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
    const Bond& bond,
    const BondConfig& config) const
{
    (void)bond; // Unused

    physx::PxFixedJoint* joint = physx::PxFixedJointCreate(
        *physics,
        actor1, frame1,
        actor2, frame2);

    if (!joint)
        return nullptr;

    // Configure break force if breakable
    if (config.breakable)
    {
        joint->setBreakForce(config.breakForce, config.breakTorque);
    }

    // Configure collision
    joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, config.enableCollision);

    return joint;
}

JointDef RigidBondType::getJointDef(const Bond& bond, const BondConfig& config) const
{
    (void)bond;

    JointDef def;
    def.type = JointDef::Type::FIXED;
    def.breakable = config.breakable;
    def.breakForce = config.breakForce;
    def.breakTorque = config.breakTorque;
    return def;
}

std::unique_ptr<IBondType> RigidBondType::clone() const
{
    return std::make_unique<RigidBondType>();
}

// =============================================================================
// CompliantBondType
// =============================================================================

CompliantBondType::CompliantBondType(float defaultStiffness, float defaultDamping)
    : m_defaultStiffness(defaultStiffness)
    , m_defaultDamping(defaultDamping)
{
}

std::string CompliantBondType::getDescription() const
{
    return "Spring-damper joint with stiffness=" + std::to_string(m_defaultStiffness) +
           " and damping=" + std::to_string(m_defaultDamping);
}

physx::PxJoint* CompliantBondType::createJoint(
    physx::PxPhysics* physics,
    physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
    physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
    const Bond& bond,
    const BondConfig& config) const
{
    (void)bond;

    physx::PxDistanceJoint* joint = physx::PxDistanceJointCreate(
        *physics,
        actor1, frame1,
        actor2, frame2);

    if (!joint)
        return nullptr;

    // Calculate rest length if not specified
    float restLength = config.restLength;
    if (restLength <= 0.0f)
    {
        // Calculate from current positions
        physx::PxVec3 pos1 = actor1->getGlobalPose().transform(frame1.p);
        physx::PxVec3 pos2 = actor2->getGlobalPose().transform(frame2.p);
        restLength = (pos2 - pos1).magnitude();
    }

    // Use config values or defaults
    float stiffness = (config.stiffness > 0.0f) ? config.stiffness : m_defaultStiffness;
    float damping = (config.damping > 0.0f) ? config.damping : m_defaultDamping;

    // Configure distance constraints
    float minDist = (config.minDistance > 0.0f) ? config.minDistance : restLength * 0.9f;
    float maxDist = (config.maxDistance > 0.0f) ? config.maxDistance : restLength * 1.1f;

    joint->setMinDistance(minDist);
    joint->setMaxDistance(maxDist);
    joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
    joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
    joint->setDistanceJointFlag(physx::PxDistanceJointFlag::eSPRING_ENABLED, true);

    joint->setStiffness(stiffness);
    joint->setDamping(damping);

    // Configure break force if breakable
    if (config.breakable)
    {
        joint->setBreakForce(config.breakForce, config.breakTorque);
    }

    joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, config.enableCollision);

    return joint;
}

JointDef CompliantBondType::getJointDef(const Bond& bond, const BondConfig& config) const
{
    (void)bond;

    JointDef def;
    def.type = JointDef::Type::DISTANCE;
    def.breakable = config.breakable;
    def.breakForce = config.breakForce;
    def.breakTorque = config.breakTorque;
    def.springStiffness = (config.stiffness > 0.0f) ? config.stiffness : m_defaultStiffness;
    def.springDamping = (config.damping > 0.0f) ? config.damping : m_defaultDamping;
    def.minDistance = config.minDistance;
    def.maxDistance = config.maxDistance;
    return def;
}

std::unique_ptr<IBondType> CompliantBondType::clone() const
{
    return std::make_unique<CompliantBondType>(m_defaultStiffness, m_defaultDamping);
}

// =============================================================================
// HingedBondType
// =============================================================================

HingedBondType::HingedBondType(
    const physx::PxVec3& axis,
    bool enableLimits,
    float lowerLimit,
    float upperLimit)
    : m_axis(axis.getNormalized())
    , m_enableLimits(enableLimits)
    , m_lowerLimit(lowerLimit)
    , m_upperLimit(upperLimit)
{
}

std::string HingedBondType::getDescription() const
{
    std::string desc = "Revolute joint allowing rotation around axis";
    if (m_enableLimits)
    {
        desc += " with limits [" + std::to_string(m_lowerLimit * 180.0f / physx::PxPi) +
                ", " + std::to_string(m_upperLimit * 180.0f / physx::PxPi) + "] degrees";
    }
    return desc;
}

physx::PxQuat HingedBondType::computeAxisRotation() const
{
    // PhysX revolute joint rotates around X axis by default
    // We need to compute rotation that aligns X with our desired axis
    physx::PxVec3 defaultAxis(1, 0, 0);

    if (std::abs(m_axis.dot(defaultAxis) - 1.0f) < 0.0001f)
        return physx::PxQuat(physx::PxIdentity);

    if (std::abs(m_axis.dot(defaultAxis) + 1.0f) < 0.0001f)
        return physx::PxQuat(physx::PxPi, physx::PxVec3(0, 1, 0));

    physx::PxVec3 rotAxis = defaultAxis.cross(m_axis).getNormalized();
    float angle = std::acos(clampValue(defaultAxis.dot(m_axis), -1.0f, 1.0f));
    return physx::PxQuat(angle, rotAxis);
}

physx::PxJoint* HingedBondType::createJoint(
    physx::PxPhysics* physics,
    physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
    physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
    const Bond& bond,
    const BondConfig& config) const
{
    (void)bond;

    // Apply axis rotation to frames
    physx::PxQuat axisRot = computeAxisRotation();
    physx::PxTransform adjustedFrame1(frame1.p, frame1.q * axisRot);
    physx::PxTransform adjustedFrame2(frame2.p, frame2.q * axisRot);

    physx::PxRevoluteJoint* joint = physx::PxRevoluteJointCreate(
        *physics,
        actor1, adjustedFrame1,
        actor2, adjustedFrame2);

    if (!joint)
        return nullptr;

    // Configure limits
    if (m_enableLimits)
    {
        joint->setLimit(physx::PxJointAngularLimitPair(m_lowerLimit, m_upperLimit));
        joint->setRevoluteJointFlag(physx::PxRevoluteJointFlag::eLIMIT_ENABLED, true);
    }

    // Configure break force if breakable
    if (config.breakable)
    {
        joint->setBreakForce(config.breakForce, config.breakTorque);
    }

    joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, config.enableCollision);

    return joint;
}

JointDef HingedBondType::getJointDef(const Bond& bond, const BondConfig& config) const
{
    (void)bond;

    JointDef def;
    def.type = JointDef::Type::REVOLUTE;
    def.breakable = config.breakable;
    def.breakForce = config.breakForce;
    def.breakTorque = config.breakTorque;
    def.limitEnabled = m_enableLimits;
    def.limitLower = m_lowerLimit;
    def.limitUpper = m_upperLimit;
    return def;
}

std::unique_ptr<IBondType> HingedBondType::clone() const
{
    return std::make_unique<HingedBondType>(m_axis, m_enableLimits, m_lowerLimit, m_upperLimit);
}

// =============================================================================
// BallSocketBondType
// =============================================================================

BallSocketBondType::BallSocketBondType(bool enableConeLimit, float coneAngle)
    : m_enableConeLimit(enableConeLimit)
    , m_coneAngle(coneAngle)
{
}

std::string BallSocketBondType::getDescription() const
{
    std::string desc = "Spherical joint allowing rotation around all axes";
    if (m_enableConeLimit)
    {
        desc += " with cone limit of " + std::to_string(m_coneAngle * 180.0f / physx::PxPi) + " degrees";
    }
    return desc;
}

physx::PxJoint* BallSocketBondType::createJoint(
    physx::PxPhysics* physics,
    physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
    physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
    const Bond& bond,
    const BondConfig& config) const
{
    (void)bond;

    physx::PxSphericalJoint* joint = physx::PxSphericalJointCreate(
        *physics,
        actor1, frame1,
        actor2, frame2);

    if (!joint)
        return nullptr;

    // Configure cone limit
    if (m_enableConeLimit)
    {
        joint->setLimitCone(physx::PxJointLimitCone(m_coneAngle, m_coneAngle));
        joint->setSphericalJointFlag(physx::PxSphericalJointFlag::eLIMIT_ENABLED, true);
    }

    // Configure break force if breakable
    if (config.breakable)
    {
        joint->setBreakForce(config.breakForce, config.breakTorque);
    }

    joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, config.enableCollision);

    return joint;
}

JointDef BallSocketBondType::getJointDef(const Bond& bond, const BondConfig& config) const
{
    (void)bond;

    JointDef def;
    def.type = JointDef::Type::SPHERICAL;
    def.breakable = config.breakable;
    def.breakForce = config.breakForce;
    def.breakTorque = config.breakTorque;
    def.limitEnabled = m_enableConeLimit;
    def.limitConeAngle = m_coneAngle;
    return def;
}

std::unique_ptr<IBondType> BallSocketBondType::clone() const
{
    return std::make_unique<BallSocketBondType>(m_enableConeLimit, m_coneAngle);
}

// =============================================================================
// PrismaticBondType
// =============================================================================

PrismaticBondType::PrismaticBondType(
    const physx::PxVec3& axis,
    bool enableLimits,
    float lowerLimit,
    float upperLimit)
    : m_axis(axis.getNormalized())
    , m_enableLimits(enableLimits)
    , m_lowerLimit(lowerLimit)
    , m_upperLimit(upperLimit)
{
}

std::string PrismaticBondType::getDescription() const
{
    std::string desc = "Prismatic joint allowing linear motion along axis";
    if (m_enableLimits)
    {
        desc += " with limits [" + std::to_string(m_lowerLimit) +
                ", " + std::to_string(m_upperLimit) + "]";
    }
    return desc;
}

physx::PxQuat PrismaticBondType::computeAxisRotation() const
{
    // PhysX prismatic joint slides along X axis by default
    physx::PxVec3 defaultAxis(1, 0, 0);

    if (std::abs(m_axis.dot(defaultAxis) - 1.0f) < 0.0001f)
        return physx::PxQuat(physx::PxIdentity);

    if (std::abs(m_axis.dot(defaultAxis) + 1.0f) < 0.0001f)
        return physx::PxQuat(physx::PxPi, physx::PxVec3(0, 1, 0));

    physx::PxVec3 rotAxis = defaultAxis.cross(m_axis).getNormalized();
    float angle = std::acos(clampValue(defaultAxis.dot(m_axis), -1.0f, 1.0f));
    return physx::PxQuat(angle, rotAxis);
}

physx::PxJoint* PrismaticBondType::createJoint(
    physx::PxPhysics* physics,
    physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
    physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
    const Bond& bond,
    const BondConfig& config) const
{
    (void)bond;

    physx::PxQuat axisRot = computeAxisRotation();
    physx::PxTransform adjustedFrame1(frame1.p, frame1.q * axisRot);
    physx::PxTransform adjustedFrame2(frame2.p, frame2.q * axisRot);

    physx::PxPrismaticJoint* joint = physx::PxPrismaticJointCreate(
        *physics,
        actor1, adjustedFrame1,
        actor2, adjustedFrame2);

    if (!joint)
        return nullptr;

    // Configure limits
    if (m_enableLimits)
    {
        physx::PxTolerancesScale scale;
        joint->setLimit(physx::PxJointLinearLimitPair(scale, m_lowerLimit, m_upperLimit));
        joint->setPrismaticJointFlag(physx::PxPrismaticJointFlag::eLIMIT_ENABLED, true);
    }

    // Configure break force if breakable
    if (config.breakable)
    {
        joint->setBreakForce(config.breakForce, config.breakTorque);
    }

    joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, config.enableCollision);

    return joint;
}

JointDef PrismaticBondType::getJointDef(const Bond& bond, const BondConfig& config) const
{
    (void)bond;

    JointDef def;
    def.type = JointDef::Type::PRISMATIC;
    def.breakable = config.breakable;
    def.breakForce = config.breakForce;
    def.breakTorque = config.breakTorque;
    def.limitEnabled = m_enableLimits;
    def.limitLower = m_lowerLimit;
    def.limitUpper = m_upperLimit;
    return def;
}

std::unique_ptr<IBondType> PrismaticBondType::clone() const
{
    return std::make_unique<PrismaticBondType>(m_axis, m_enableLimits, m_lowerLimit, m_upperLimit);
}

// =============================================================================
// D6BondType
// =============================================================================

D6BondType::D6BondType()
{
    // Default: all axes locked (rigid behavior)
}

std::string D6BondType::getDescription() const
{
    return "Fully configurable D6 joint with per-axis motion control";
}

physx::PxJoint* D6BondType::createJoint(
    physx::PxPhysics* physics,
    physx::PxRigidActor* actor1, const physx::PxTransform& frame1,
    physx::PxRigidActor* actor2, const physx::PxTransform& frame2,
    const Bond& bond,
    const BondConfig& config) const
{
    (void)bond;

    physx::PxD6Joint* joint = physx::PxD6JointCreate(
        *physics,
        actor1, frame1,
        actor2, frame2);

    if (!joint)
        return nullptr;

    // Configure linear axes
    joint->setMotion(physx::PxD6Axis::eX, m_linearX.motion);
    joint->setMotion(physx::PxD6Axis::eY, m_linearY.motion);
    joint->setMotion(physx::PxD6Axis::eZ, m_linearZ.motion);

    // Configure angular axes
    joint->setMotion(physx::PxD6Axis::eTWIST, m_twist.motion);
    joint->setMotion(physx::PxD6Axis::eSWING1, m_swing1.motion);
    joint->setMotion(physx::PxD6Axis::eSWING2, m_swing2.motion);

    // Configure linear limits if needed
    if (m_linearX.motion == physx::PxD6Motion::eLIMITED ||
        m_linearY.motion == physx::PxD6Motion::eLIMITED ||
        m_linearZ.motion == physx::PxD6Motion::eLIMITED)
    {
        // Use the maximum of all linear limits
        float maxLimit = std::max({
            std::abs(m_linearX.limitUpper), std::abs(m_linearX.limitLower),
            std::abs(m_linearY.limitUpper), std::abs(m_linearY.limitLower),
            std::abs(m_linearZ.limitUpper), std::abs(m_linearZ.limitLower)
        });
        (void)maxLimit; // Suppress unused variable warning
        physx::PxTolerancesScale scale;
        joint->setLinearLimit(physx::PxD6Axis::eX, physx::PxJointLinearLimitPair(scale, m_linearX.limitLower, m_linearX.limitUpper));
        joint->setLinearLimit(physx::PxD6Axis::eY, physx::PxJointLinearLimitPair(scale, m_linearY.limitLower, m_linearY.limitUpper));
        joint->setLinearLimit(physx::PxD6Axis::eZ, physx::PxJointLinearLimitPair(scale, m_linearZ.limitLower, m_linearZ.limitUpper));
    }

    // Configure angular limits
    if (m_twist.motion == physx::PxD6Motion::eLIMITED)
    {
        joint->setTwistLimit(physx::PxJointAngularLimitPair(m_twist.limitLower, m_twist.limitUpper));
    }

    if (m_swing1.motion == physx::PxD6Motion::eLIMITED ||
        m_swing2.motion == physx::PxD6Motion::eLIMITED)
    {
        joint->setSwingLimit(physx::PxJointLimitCone(
            (m_swing1.motion == physx::PxD6Motion::eLIMITED) ? m_swing1.limitUpper : physx::PxPi,
            (m_swing2.motion == physx::PxD6Motion::eLIMITED) ? m_swing2.limitUpper : physx::PxPi));
    }

    // Configure drives
    if (m_linearX.driveStiffness > 0 || m_linearX.driveDamping > 0)
    {
        joint->setDrive(physx::PxD6Drive::eX,
            physx::PxD6JointDrive(m_linearX.driveStiffness, m_linearX.driveDamping, PX_MAX_F32));
    }
    if (m_linearY.driveStiffness > 0 || m_linearY.driveDamping > 0)
    {
        joint->setDrive(physx::PxD6Drive::eY,
            physx::PxD6JointDrive(m_linearY.driveStiffness, m_linearY.driveDamping, PX_MAX_F32));
    }
    if (m_linearZ.driveStiffness > 0 || m_linearZ.driveDamping > 0)
    {
        joint->setDrive(physx::PxD6Drive::eZ,
            physx::PxD6JointDrive(m_linearZ.driveStiffness, m_linearZ.driveDamping, PX_MAX_F32));
    }
    if (m_twist.driveStiffness > 0 || m_twist.driveDamping > 0)
    {
        joint->setDrive(physx::PxD6Drive::eTWIST,
            physx::PxD6JointDrive(m_twist.driveStiffness, m_twist.driveDamping, PX_MAX_F32));
    }
    if (m_swing1.driveStiffness > 0 || m_swing1.driveDamping > 0 ||
        m_swing2.driveStiffness > 0 || m_swing2.driveDamping > 0)
    {
        float swingStiff = std::max(m_swing1.driveStiffness, m_swing2.driveStiffness);
        float swingDamp = std::max(m_swing1.driveDamping, m_swing2.driveDamping);
        joint->setDrive(physx::PxD6Drive::eSWING,
            physx::PxD6JointDrive(swingStiff, swingDamp, PX_MAX_F32));
    }

    // Configure break force if breakable
    if (config.breakable)
    {
        joint->setBreakForce(config.breakForce, config.breakTorque);
    }

    joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, config.enableCollision);

    return joint;
}

JointDef D6BondType::getJointDef(const Bond& bond, const BondConfig& config) const
{
    (void)bond;

    JointDef def;
    def.type = JointDef::Type::D6;
    def.breakable = config.breakable;
    def.breakForce = config.breakForce;
    def.breakTorque = config.breakTorque;
    return def;
}

std::unique_ptr<IBondType> D6BondType::clone() const
{
    auto clone = std::make_unique<D6BondType>();
    clone->m_linearX = m_linearX;
    clone->m_linearY = m_linearY;
    clone->m_linearZ = m_linearZ;
    clone->m_twist = m_twist;
    clone->m_swing1 = m_swing1;
    clone->m_swing2 = m_swing2;
    return clone;
}

} // namespace bonding
