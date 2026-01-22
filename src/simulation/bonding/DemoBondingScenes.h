#pragma once

#include "BondingIntegration.h"
#include <random>
#include <iostream>

namespace bonding
{
namespace demo
{

/// Create a basic dynamic bonding demo scene
/// Particles move around and form bonds when they get close
inline void createDynamicBondingScene(
    physx::PxPhysics* physics,
    physx::PxScene* scene,
    physx::PxMaterial* material,
    DynamicBondManager& bondManager,
    uint32_t seed = 42)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> posDist(-10.0f, 10.0f);
    std::uniform_real_distribution<float> velDist(-2.0f, 2.0f);

    // Setup bond manager with molecular rules
    setupDefaultBondManager(bondManager, physics, scene, 2.0f);

    // Clear default rules and add molecular-style rules
    bondManager.clearRules();
    for (auto& rule : createMolecularRules(2.0f, 0.5f))
    {
        bondManager.addRule(rule);
    }

    // Use compliant bonds
    bondManager.setDefaultBondType("compliant");

    // Create ground plane
    physx::PxRigidStatic* ground = physics->createRigidStatic(
        physx::PxTransform(physx::PxVec3(0.0f, 0.0f, 0.0f)));
    physx::PxShape* groundShape = physics->createShape(
        physx::PxPlaneGeometry(), *material);
    ground->attachShape(*groundShape);
    groundShape->release();
    ground->setGlobalPose(physx::PxTransform(
        physx::PxVec3(0.0f, 0.0f, 0.0f),
        physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0.0f, 0.0f, 1.0f))));
    scene->addActor(*ground);

    // Create bondable particles (dimers)
    const int numParticles = 20;

    for (int i = 0; i < numParticles; ++i)
    {
        // Create entity definition
        BondableEntityDef def = BondableEntityDef::createDimer(0.5f);
        def.entityType = "monomer";

        // Random position
        def.actorDef.posX = posDist(rng);
        def.actorDef.posY = 5.0f + posDist(rng) * 0.3f;
        def.actorDef.posZ = posDist(rng);

        // Random velocity
        def.actorDef.velX = velDist(rng);
        def.actorDef.velY = 0.0f;
        def.actorDef.velZ = velDist(rng);

        // Physics properties
        def.actorDef.density = 1.0f;
        def.actorDef.staticFriction = material->getStaticFriction();
        def.actorDef.dynamicFriction = material->getDynamicFriction();
        def.actorDef.restitution = material->getRestitution();

        bondManager.registerEntity(def);
    }

    // Add callback to log bond formation
    bondManager.onBondFormed([](const BondFormedEvent& event) {
        std::cout << "Bond formed: " << event.bond.bondId
                  << " between entities " << event.bond.endpoint1.entityId
                  << " and " << event.bond.endpoint2.entityId
                  << " at time " << event.simulationTime << std::endl;
    });

    std::cout << "Dynamic bonding scene created with " << numParticles << " particles" << std::endl;
}

/// Create a ring formation demo scene
/// Particles try to form closed rings based on angle constraints
inline void createRingFormationScene(
    physx::PxPhysics* physics,
    physx::PxScene* scene,
    physx::PxMaterial* material,
    DynamicBondManager& bondManager,
    int targetRingSize = 4,
    uint32_t seed = 42)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> posDist(-5.0f, 5.0f);

    // Setup bond manager with ring formation rules
    setupDefaultBondManager(bondManager, physics, scene, 0.8f);

    // Configure for ring formation
    bondManager.clearRules();
    float targetAngle = physx::PxPi * (1.0f - 2.0f / static_cast<float>(targetRingSize));
    for (auto& rule : createRingFormationRules(0.8f, targetAngle, 0.1f))
    {
        bondManager.addRule(rule);
    }

    // Use compliant bonds for better dynamics
    bondManager.setDefaultBondType("compliant");

    // Create ground plane
    physx::PxRigidStatic* ground = physics->createRigidStatic(
        physx::PxTransform(physx::PxVec3(0.0f, 0.0f, 0.0f)));
    physx::PxShape* groundShape = physics->createShape(
        physx::PxPlaneGeometry(), *material);
    ground->attachShape(*groundShape);
    groundShape->release();
    ground->setGlobalPose(physx::PxTransform(
        physx::PxVec3(0.0f, 0.0f, 0.0f),
        physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0.0f, 0.0f, 1.0f))));
    scene->addActor(*ground);

    // Create monomers that can form rings
    const int numMonomers = targetRingSize * 3; // 3x the ring size

    for (int i = 0; i < numMonomers; ++i)
    {
        BondableEntityDef def = BondableEntityDef::createDimer(0.4f);
        def.entityType = "ring_monomer";

        // Arrange in a plane
        def.actorDef.posX = posDist(rng);
        def.actorDef.posY = 5.0f;
        def.actorDef.posZ = posDist(rng);

        // Small random rotation
        float angle = static_cast<float>(rng()) / static_cast<float>(rng.max()) * physx::PxTwoPi;
        def.actorDef.quatW = std::cos(angle / 2.0f);
        def.actorDef.quatX = 0.0f;
        def.actorDef.quatY = std::sin(angle / 2.0f);
        def.actorDef.quatZ = 0.0f;

        def.actorDef.density = 1.0f;
        def.actorDef.staticFriction = 0.5f;
        def.actorDef.dynamicFriction = 0.5f;
        def.actorDef.restitution = 0.3f;

        bondManager.registerEntity(def);
    }

    std::cout << "Ring formation scene created with " << numMonomers
              << " monomers, target ring size: " << targetRingSize << std::endl;
}

/// Create a chain formation demo
/// Particles form linear chains
inline void createChainFormationScene(
    physx::PxPhysics* physics,
    physx::PxScene* scene,
    physx::PxMaterial* material,
    DynamicBondManager& bondManager,
    uint32_t seed = 42)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> posDist(-8.0f, 8.0f);

    // Setup bond manager
    setupDefaultBondManager(bondManager, physics, scene, 1.5f);

    // Linear chain rules (no angle constraint, just proximity and valency)
    bondManager.clearRules();
    bondManager.addRule(std::make_shared<NoSelfBondingRule>());
    bondManager.addRule(std::make_shared<NoDuplicateBondRule>());
    bondManager.addRule(std::make_shared<ValencyRule>());
    bondManager.addRule(std::make_shared<ProximityRule>(1.5f));
    bondManager.addRule(std::make_shared<DirectionalAlignmentRule>(
        DirectionalAlignmentRule::AlignmentMode::ANTIPARALLEL, 0.8f));

    // Use rigid bonds for chains
    bondManager.setDefaultBondType("rigid");

    // Create ground
    physx::PxRigidStatic* ground = physics->createRigidStatic(
        physx::PxTransform(physx::PxVec3(0.0f, 0.0f, 0.0f)));
    physx::PxShape* groundShape = physics->createShape(
        physx::PxPlaneGeometry(), *material);
    ground->attachShape(*groundShape);
    groundShape->release();
    ground->setGlobalPose(physx::PxTransform(
        physx::PxVec3(0.0f, 0.0f, 0.0f),
        physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0.0f, 0.0f, 1.0f))));
    scene->addActor(*ground);

    // Create chain monomers (each can bond at most 2 sites - linear chain)
    const int numMonomers = 30;

    for (int i = 0; i < numMonomers; ++i)
    {
        BondableEntityDef def = BondableEntityDef::createDimer(0.3f);
        def.entityType = "chain_monomer";

        // Random positions
        def.actorDef.posX = posDist(rng);
        def.actorDef.posY = 3.0f + static_cast<float>(i % 5);
        def.actorDef.posZ = posDist(rng);

        def.actorDef.density = 1.0f;

        bondManager.registerEntity(def);
    }

    std::cout << "Chain formation scene created with " << numMonomers << " monomers" << std::endl;
}

/// Create a batch simulation example
/// Returns results from running multiple simulation replicates
inline std::vector<batch::SimulationResult> runBatchSimulation(
    physx::PxPhysics* physics,
    physx::PxMaterial* material,
    int numReplicates = 10,
    float maxTime = 30.0f)
{
    // Create a temporary bond manager for metrics (will be replaced in each replicate)
    auto bondManager = std::make_unique<DynamicBondManager>();

    // Create batch runner
    auto runner = createStandardBatchRunner(bondManager.get(), numReplicates, maxTime);

    // Add ring formation metric
    runner->addMetric(std::make_shared<batch::RingFormationMetric>(bondManager.get(), 4));

    // Add ring formation termination condition
    runner->addTerminationCondition(
        std::make_shared<batch::RingFormationCondition>(bondManager.get(), 4));

    // Define scene factory
    batch::SceneFactory sceneFactory = [material](
        physx::PxPhysics* physics,
        physx::PxScene* scene,
        DynamicBondManager* manager,
        uint32_t seed)
    {
        createRingFormationScene(physics, scene, material, *manager, 4, seed);
    };

    // Run batch
    std::cout << "Starting batch simulation with " << numReplicates << " replicates..." << std::endl;
    auto results = runner->run(physics, sceneFactory);

    // Calculate and print summary
    auto summary = batch::BatchSimulationRunner::calculateSummary(results);
    std::cout << "\nBatch Simulation Summary:" << std::endl;
    std::cout << "  Total replicates: " << summary.totalReplicates << std::endl;
    std::cout << "  Successful: " << summary.successfulReplicates << std::endl;
    std::cout << "  Mean time: " << summary.meanTime << " +/- " << summary.stdTime << " s" << std::endl;
    std::cout << "  Time range: [" << summary.minTime << ", " << summary.maxTime << "] s" << std::endl;

    std::cout << "\n  Termination reasons:" << std::endl;
    for (const auto& [reason, count] : summary.terminationReasonCounts)
    {
        std::cout << "    " << reason << ": " << count << std::endl;
    }

    return results;
}

} // namespace demo
} // namespace bonding
