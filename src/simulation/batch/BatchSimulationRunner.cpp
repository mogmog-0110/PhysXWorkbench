#include "BatchSimulationRunner.h"
#include "CommonTerminationConditions.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace batch
{

BatchSimulationRunner::BatchSimulationRunner()
{
    // Add default timeout condition
    addTerminationCondition(std::make_shared<TimeoutCondition>(60.0f));
}

BatchSimulationRunner::~BatchSimulationRunner()
{
    cancel();
}

void BatchSimulationRunner::configure(const BatchConfig& config)
{
    m_config = config;

    // Update timeout condition if it exists
    for (auto& cond : m_conditions)
    {
        if (auto* timeout = dynamic_cast<TimeoutCondition*>(cond.get()))
        {
            timeout->setMaxTime(config.maxSimulationTime);
            break;
        }
    }
}

void BatchSimulationRunner::addMetric(MetricPtr metric)
{
    m_metrics.push_back(std::move(metric));
}

void BatchSimulationRunner::removeMetric(const std::string& name)
{
    m_metrics.erase(
        std::remove_if(m_metrics.begin(), m_metrics.end(),
            [&name](const MetricPtr& metric)
            {
                return metric->getName() == name;
            }),
        m_metrics.end());
}

void BatchSimulationRunner::clearMetrics()
{
    m_metrics.clear();
}

void BatchSimulationRunner::addTerminationCondition(TerminationConditionPtr condition)
{
    m_conditions.push_back(std::move(condition));
}

void BatchSimulationRunner::removeTerminationCondition(const std::string& name)
{
    m_conditions.erase(
        std::remove_if(m_conditions.begin(), m_conditions.end(),
            [&name](const TerminationConditionPtr& cond)
            {
                return cond->getName() == name;
            }),
        m_conditions.end());
}

void BatchSimulationRunner::clearTerminationConditions()
{
    m_conditions.clear();
}

std::vector<SimulationResult> BatchSimulationRunner::run(
    physx::PxPhysics* physics,
    SceneFactory sceneFactory)
{
    m_running.store(true);
    m_cancelled.store(false);

    std::vector<SimulationResult> results;
    results.reserve(m_config.numReplicates);

    for (int i = 0; i < m_config.numReplicates && !m_cancelled.load(); ++i)
    {
        uint32_t seed = m_config.baseSeed + static_cast<uint32_t>(i);
        auto result = runSingleReplicate(physics, sceneFactory, i, seed);
        results.push_back(std::move(result));

        // Progress callback
        if (m_config.progressCallback)
        {
            m_config.progressCallback(i + 1, m_config.numReplicates);
        }
    }

    m_running.store(false);
    return results;
}

SimulationResult BatchSimulationRunner::runSingleReplicate(
    physx::PxPhysics* physics,
    SceneFactory sceneFactory,
    int replicateId,
    uint32_t seed)
{
    SimulationResult result;
    result.replicateId = replicateId;
    result.seed = seed;

    try
    {
        // Create scene
        physx::PxScene* scene = createScene(physics);
        if (!scene)
        {
            result.success = false;
            result.errorMessage = "Failed to create PhysX scene";
            return result;
        }

        // Create bond manager
        auto bondManager = std::make_unique<bonding::DynamicBondManager>();
        bondManager->initialize(physics, scene);

        // Reset metrics and conditions
        for (auto& metric : m_metrics)
        {
            metric->reset();
        }
        for (auto& cond : m_conditions)
        {
            cond->reset();
        }

        // Setup scene using factory
        sceneFactory(physics, scene, bondManager.get(), seed);

        // Run simulation
        runSimulationLoop(scene, bondManager.get(), result);

        // Collect final metrics
        collectFinalMetrics(result);

        // Cleanup
        bondManager->releaseAll();
        scene->release();

        result.success = true;
    }
    catch (const std::exception& e)
    {
        result.success = false;
        result.errorMessage = e.what();
    }

    return result;
}

void BatchSimulationRunner::cancel()
{
    m_cancelled.store(true);
}

physx::PxScene* BatchSimulationRunner::createScene(physx::PxPhysics* physics)
{
    physx::PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);

    // Create CPU dispatcher
    physx::PxDefaultCpuDispatcher* dispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    if (!dispatcher)
        return nullptr;

    sceneDesc.cpuDispatcher = dispatcher;
    sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;

    return physics->createScene(sceneDesc);
}

void BatchSimulationRunner::runSimulationLoop(
    physx::PxScene* scene,
    bonding::DynamicBondManager* bondManager,
    SimulationResult& result)
{
    float time = 0.0f;
    float metricUpdateAccum = 0.0f;
    result.totalTime = 0.0f;

    while (!m_cancelled.load())
    {
        // Step physics
        scene->simulate(m_config.timestep);
        scene->fetchResults(true);

        // Update bond manager
        bondManager->update(m_config.timestep);

        time += m_config.timestep;
        metricUpdateAccum += m_config.timestep;

        // Update metrics
        if (m_config.metricUpdateInterval <= 0.0f ||
            metricUpdateAccum >= m_config.metricUpdateInterval)
        {
            updateMetrics(scene, time, metricUpdateAccum);
            metricUpdateAccum = 0.0f;
        }

        // Check termination
        std::string terminationReason = checkTermination(scene, time);
        if (!terminationReason.empty())
        {
            result.terminationReason = terminationReason;
            result.totalTime = time;
            break;
        }

        // Safety timeout
        if (time >= m_config.maxSimulationTime)
        {
            result.terminationReason = "max_time_reached";
            result.totalTime = time;
            break;
        }
    }

    // Final metric update
    updateMetrics(scene, time, 0.0f);
}

std::string BatchSimulationRunner::checkTermination(physx::PxScene* scene, float time)
{
    for (const auto& cond : m_conditions)
    {
        if (cond->shouldTerminate(scene, time))
        {
            return cond->getName();
        }
    }
    return "";
}

void BatchSimulationRunner::updateMetrics(physx::PxScene* scene, float time, float dt)
{
    for (auto& metric : m_metrics)
    {
        metric->update(scene, time, dt);
    }
}

void BatchSimulationRunner::collectFinalMetrics(SimulationResult& result)
{
    for (const auto& metric : m_metrics)
    {
        result.finalMetrics[metric->getName()] = metric->getValue();

        auto timeSeries = metric->getTimeSeries();
        if (!timeSeries.empty())
        {
            result.timeSeries[metric->getName()] = timeSeries;
        }
    }
}

bool BatchSimulationRunner::exportToCSV(
    const std::vector<SimulationResult>& results,
    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
        return false;

    // Collect all metric names
    std::vector<std::string> metricNames;
    if (!results.empty())
    {
        for (const auto& [name, value] : results[0].finalMetrics)
        {
            metricNames.push_back(name);
        }
    }

    // Write header
    file << "replicate_id,seed,total_time,termination_reason,success";
    for (const auto& name : metricNames)
    {
        file << "," << name;
    }
    file << "\n";

    // Write data rows
    for (const auto& result : results)
    {
        file << result.replicateId << ","
             << result.seed << ","
             << std::fixed << std::setprecision(4) << result.totalTime << ","
             << result.terminationReason << ","
             << (result.success ? "true" : "false");

        for (const auto& name : metricNames)
        {
            file << ",";
            auto it = result.finalMetrics.find(name);
            if (it != result.finalMetrics.end())
            {
                file << metricValueToString(it->second);
            }
        }
        file << "\n";
    }

    return true;
}

bool BatchSimulationRunner::exportToJSON(
    const std::vector<SimulationResult>& results,
    const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
        return false;

    file << "{\n";
    file << "  \"results\": [\n";

    for (size_t i = 0; i < results.size(); ++i)
    {
        const auto& result = results[i];

        file << "    {\n";
        file << "      \"replicate_id\": " << result.replicateId << ",\n";
        file << "      \"seed\": " << result.seed << ",\n";
        file << "      \"total_time\": " << std::fixed << std::setprecision(4) << result.totalTime << ",\n";
        file << "      \"termination_reason\": \"" << result.terminationReason << "\",\n";
        file << "      \"success\": " << (result.success ? "true" : "false") << ",\n";

        // Metrics
        file << "      \"metrics\": {\n";
        size_t metricCount = 0;
        for (const auto& [name, value] : result.finalMetrics)
        {
            file << "        \"" << name << "\": " << metricValueToString(value);
            if (++metricCount < result.finalMetrics.size())
                file << ",";
            file << "\n";
        }
        file << "      }";

        // Time series (if any)
        if (!result.timeSeries.empty())
        {
            file << ",\n      \"time_series\": {\n";
            size_t tsCount = 0;
            for (const auto& [name, series] : result.timeSeries)
            {
                file << "        \"" << name << "\": [\n";
                for (size_t j = 0; j < series.size(); ++j)
                {
                    file << "          [" << series[j].first << ", "
                         << metricValueToString(series[j].second) << "]";
                    if (j + 1 < series.size())
                        file << ",";
                    file << "\n";
                }
                file << "        ]";
                if (++tsCount < result.timeSeries.size())
                    file << ",";
                file << "\n";
            }
            file << "      }";
        }

        file << "\n    }";
        if (i + 1 < results.size())
            file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";

    return true;
}

BatchSimulationRunner::SummaryStats BatchSimulationRunner::calculateSummary(
    const std::vector<SimulationResult>& results)
{
    SummaryStats stats;
    stats.totalReplicates = static_cast<int>(results.size());

    if (results.empty())
        return stats;

    // Calculate time statistics
    std::vector<float> times;
    for (const auto& result : results)
    {
        if (result.success)
        {
            stats.successfulReplicates++;
            times.push_back(result.totalTime);
        }
        stats.terminationReasonCounts[result.terminationReason]++;
    }

    if (!times.empty())
    {
        stats.minTime = *std::min_element(times.begin(), times.end());
        stats.maxTime = *std::max_element(times.begin(), times.end());
        stats.meanTime = std::accumulate(times.begin(), times.end(), 0.0f) /
                         static_cast<float>(times.size());

        // Standard deviation
        float variance = 0.0f;
        for (float t : times)
        {
            float diff = t - stats.meanTime;
            variance += diff * diff;
        }
        variance /= static_cast<float>(times.size());
        stats.stdTime = std::sqrt(variance);
    }

    // Calculate mean metric values (for numeric metrics)
    std::unordered_map<std::string, std::vector<float>> metricValues;

    for (const auto& result : results)
    {
        if (!result.success)
            continue;

        for (const auto& [name, value] : result.finalMetrics)
        {
            float floatValue = 0.0f;
            bool isNumeric = false;

            if (auto* f = std::get_if<float>(&value))
            {
                floatValue = *f;
                isNumeric = true;
            }
            else if (auto* d = std::get_if<double>(&value))
            {
                floatValue = static_cast<float>(*d);
                isNumeric = true;
            }
            else if (auto* i = std::get_if<int>(&value))
            {
                floatValue = static_cast<float>(*i);
                isNumeric = true;
            }

            if (isNumeric)
            {
                metricValues[name].push_back(floatValue);
            }
        }
    }

    for (const auto& [name, values] : metricValues)
    {
        if (!values.empty())
        {
            stats.meanMetricValues[name] = std::accumulate(values.begin(), values.end(), 0.0f) /
                                           static_cast<float>(values.size());
        }
    }

    return stats;
}

} // namespace batch
