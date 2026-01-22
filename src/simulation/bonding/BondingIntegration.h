#pragma once

/// Bonding System Integration Header
/// Include this file to get access to the complete dynamic bonding system

// Core components
#include "PropertyMap.h"
#include "BondingSiteDef.h"
#include "BondableEntityDef.h"
#include "BondableEntity.h"
#include "Bond.h"

// Rules
#include "IBondFormationRule.h"
#include "BondFormationRules.h"

// Bond types
#include "IBondType.h"
#include "BondTypes.h"

// Manager
#include "DynamicBondManager.h"

// Batch simulation
#include "../batch/IMetric.h"
#include "../batch/CommonMetrics.h"
#include "../batch/ITerminationCondition.h"
#include "../batch/CommonTerminationConditions.h"
#include "../batch/BatchSimulationRunner.h"

namespace bonding
{

/// Helper functions for common setups

/// Create a standard set of rules for molecular-style bonding
inline std::vector<BondFormationRulePtr> createMolecularRules(
    float captureDistance = 2.0f,
    float angleTolerance = 0.52f)  // ~30 degrees
{
    std::vector<BondFormationRulePtr> rules;
    rules.push_back(std::make_shared<NoSelfBondingRule>());
    rules.push_back(std::make_shared<NoDuplicateBondRule>());
    rules.push_back(std::make_shared<ValencyRule>());
    rules.push_back(std::make_shared<ProximityRule>(captureDistance));
    rules.push_back(std::make_shared<TypeCompatibilityRule>());
    rules.push_back(std::make_shared<DirectionalAlignmentRule>(
        DirectionalAlignmentRule::AlignmentMode::ANTIPARALLEL, angleTolerance));
    return rules;
}

/// Create rules suitable for ring formation
inline std::vector<BondFormationRulePtr> createRingFormationRules(
    float captureDistance = 0.8f,
    float targetAngle = physx::PxPi / 2.0f,  // 90 degrees for square rings
    float angleTolerance = 0.087f)  // ~5 degrees
{
    std::vector<BondFormationRulePtr> rules;
    rules.push_back(std::make_shared<NoSelfBondingRule>());
    rules.push_back(std::make_shared<NoDuplicateBondRule>());
    rules.push_back(std::make_shared<ValencyRule>());
    rules.push_back(std::make_shared<ProximityRule>(captureDistance));
    rules.push_back(std::make_shared<AngleConstraintRule>(targetAngle, angleTolerance));
    rules.push_back(std::make_shared<CoplanarityRule>(angleTolerance));
    return rules;
}

/// Setup a bond manager with default configuration
inline void setupDefaultBondManager(
    DynamicBondManager& manager,
    physx::PxPhysics* physics,
    physx::PxScene* scene,
    float captureDistance = 2.0f)
{
    manager.initialize(physics, scene);

    DynamicBondManagerConfig config;
    config.captureDistance = captureDistance;
    config.proximityCheckInterval = 0.1f;
    config.maxBondsPerFrame = 10;
    config.enableSpatialHashing = true;
    manager.configure(config);
}

/// Create a batch runner with standard metrics
inline std::unique_ptr<batch::BatchSimulationRunner> createStandardBatchRunner(
    DynamicBondManager* bondManager,
    int numReplicates = 10,
    float maxTime = 60.0f)
{
    auto runner = std::make_unique<batch::BatchSimulationRunner>();

    batch::BatchConfig config;
    config.numReplicates = numReplicates;
    config.maxSimulationTime = maxTime;
    runner->configure(config);

    // Add standard metrics
    runner->addMetric(std::make_shared<batch::BondCountMetric>(bondManager, true));
    runner->addMetric(std::make_shared<batch::KineticEnergyMetric>(true));
    runner->addMetric(std::make_shared<batch::EntityCountMetric>(bondManager));
    runner->addMetric(std::make_shared<batch::AvailableSiteCountMetric>(bondManager));

    // Add termination conditions
    runner->addTerminationCondition(std::make_shared<batch::TimeoutCondition>(maxTime));
    runner->addTerminationCondition(std::make_shared<batch::SteadyStateCondition>(0.1f, 2.0f));

    return runner;
}

} // namespace bonding
