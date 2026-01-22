#include "MSERSteadyStateCondition.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <limits>

namespace batch
{

MSERSteadyStateCondition::MSERSteadyStateCondition(
    MetricPtr metric,
    const MSERConfig& config)
    : m_metric(std::move(metric))
    , m_config(config)
{
}

std::string MSERSteadyStateCondition::getDescription() const
{
    return "MSER-based steady state detection with threshold " +
           std::to_string(m_config.convergenceThreshold) +
           " and hold time " + std::to_string(m_config.holdTime) + "s";
}

bool MSERSteadyStateCondition::shouldTerminate(physx::PxScene* scene, float time) const
{
    // Get current metric value
    MetricValue value = m_metric->getValue();

    // Extract double value
    double doubleValue = 0.0;
    if (auto* f = std::get_if<float>(&value))
        doubleValue = static_cast<double>(*f);
    else if (auto* d = std::get_if<double>(&value))
        doubleValue = *d;
    else if (auto* i = std::get_if<int>(&value))
        doubleValue = static_cast<double>(*i);
    else
        return false; // Can't track non-numeric

    // Add sample
    m_samples.push_back(doubleValue);

    float dt = time - m_lastTime;
    m_lastTime = time;

    // Not enough samples yet
    if (m_samples.size() < m_config.minSamples)
        return false;

    // Check convergence periodically
    if (m_samples.size() % m_config.checkInterval == 0)
    {
        if (checkConvergence())
        {
            m_timeBelowThreshold += dt;
        }
        else
        {
            m_timeBelowThreshold = 0.0f;
        }
    }
    else
    {
        // Continue accumulating time if already below threshold
        if (m_reachedSteadyState)
        {
            m_timeBelowThreshold += dt;
        }
    }

    return m_timeBelowThreshold >= m_config.holdTime;
}

void MSERSteadyStateCondition::reset()
{
    m_samples.clear();
    m_truncationPoint = 0;
    m_reachedSteadyState = false;
    m_steadyStateMean = 0.0;
    m_timeBelowThreshold = 0.0f;
    m_lastTime = 0.0f;
    m_metric->reset();
}

std::unique_ptr<ITerminationCondition> MSERSteadyStateCondition::clone() const
{
    return std::make_unique<MSERSteadyStateCondition>(m_metric, m_config);
}

double MSERSteadyStateCondition::computeMSER(size_t truncationPoint) const
{
    if (truncationPoint >= m_samples.size())
        return std::numeric_limits<double>::max();

    size_t n = m_samples.size() - truncationPoint;
    if (n < 2)
        return std::numeric_limits<double>::max();

    // Compute mean of retained samples
    double sum = 0.0;
    for (size_t i = truncationPoint; i < m_samples.size(); ++i)
    {
        sum += m_samples[i];
    }
    double mean = sum / static_cast<double>(n);

    // Compute variance
    double variance = 0.0;
    for (size_t i = truncationPoint; i < m_samples.size(); ++i)
    {
        double diff = m_samples[i] - mean;
        variance += diff * diff;
    }
    variance /= static_cast<double>(n - 1);

    // MSER = variance / n (marginal standard error squared)
    return variance / static_cast<double>(n);
}

size_t MSERSteadyStateCondition::findOptimalTruncationPoint() const
{
    if (m_samples.size() < m_config.minSamples)
        return 0;

    double minMSER = std::numeric_limits<double>::max();
    size_t optimalPoint = 0;

    // Search for optimal truncation point
    // Don't truncate more than half the data
    size_t maxTruncation = m_samples.size() / 2;

    for (size_t d = 0; d <= maxTruncation; ++d)
    {
        double mser = computeMSER(d);
        if (mser < minMSER)
        {
            minMSER = mser;
            optimalPoint = d;
        }
    }

    return optimalPoint;
}

bool MSERSteadyStateCondition::checkConvergence() const
{
    if (m_samples.size() < m_config.minSamples)
        return false;

    // Find optimal truncation point
    m_truncationPoint = findOptimalTruncationPoint();

    // Compute statistics after truncation
    size_t n = m_samples.size() - m_truncationPoint;
    if (n < 2)
        return false;

    // Compute mean
    double sum = 0.0;
    for (size_t i = m_truncationPoint; i < m_samples.size(); ++i)
    {
        sum += m_samples[i];
    }
    m_steadyStateMean = sum / static_cast<double>(n);

    // Compute variance
    double variance = 0.0;
    for (size_t i = m_truncationPoint; i < m_samples.size(); ++i)
    {
        double diff = m_samples[i] - m_steadyStateMean;
        variance += diff * diff;
    }
    variance /= static_cast<double>(n - 1);

    // Compute coefficient of variation (normalized variance)
    double cv = 0.0;
    if (std::abs(m_steadyStateMean) > 1e-10)
    {
        cv = std::sqrt(variance) / std::abs(m_steadyStateMean);
    }
    else
    {
        // If mean is near zero, use absolute variance threshold
        cv = std::sqrt(variance);
    }

    // Check if converged
    m_reachedSteadyState = (cv < m_config.convergenceThreshold);

    return m_reachedSteadyState;
}

} // namespace batch
