#pragma once

#include <PxPhysicsAPI.h>
#include <string>
#include <variant>
#include <vector>
#include <memory>

namespace batch
{

/// Possible metric value types
using MetricValue = std::variant<
    bool,
    int,
    float,
    double,
    std::string,
    std::vector<float>,
    std::vector<int>
>;

/// Interface for simulation metrics
/// Metrics track values throughout a simulation and provide final results
class IMetric
{
public:
    virtual ~IMetric() = default;

    /// Get the unique name of this metric
    virtual std::string getName() const = 0;

    /// Get a description of what this metric measures
    virtual std::string getDescription() const { return ""; }

    /// Update the metric with the current simulation state
    /// @param scene PhysX scene
    /// @param time Current simulation time
    /// @param dt Time step since last update
    virtual void update(physx::PxScene* scene, float time, float dt) = 0;

    /// Get the current/final value of the metric
    virtual MetricValue getValue() const = 0;

    /// Get a time series of values (if applicable)
    /// Returns empty vector if time series not tracked
    virtual std::vector<std::pair<float, MetricValue>> getTimeSeries() const
    {
        return {};
    }

    /// Reset the metric for a new simulation run
    virtual void reset() = 0;

    /// Clone this metric
    virtual std::unique_ptr<IMetric> clone() const = 0;
};

/// Shared pointer type for metrics
using MetricPtr = std::shared_ptr<IMetric>;

/// Helper to convert MetricValue to string for output
inline std::string metricValueToString(const MetricValue& value)
{
    return std::visit([](auto&& arg) -> std::string
    {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, bool>)
            return arg ? "true" : "false";
        else if constexpr (std::is_same_v<T, int>)
            return std::to_string(arg);
        else if constexpr (std::is_same_v<T, float>)
            return std::to_string(arg);
        else if constexpr (std::is_same_v<T, double>)
            return std::to_string(arg);
        else if constexpr (std::is_same_v<T, std::string>)
            return arg;
        else if constexpr (std::is_same_v<T, std::vector<float>>)
        {
            std::string result = "[";
            for (size_t i = 0; i < arg.size(); ++i)
            {
                if (i > 0) result += ",";
                result += std::to_string(arg[i]);
            }
            return result + "]";
        }
        else if constexpr (std::is_same_v<T, std::vector<int>>)
        {
            std::string result = "[";
            for (size_t i = 0; i < arg.size(); ++i)
            {
                if (i > 0) result += ",";
                result += std::to_string(arg[i]);
            }
            return result + "]";
        }
        else
            return "unknown";
    }, value);
}

} // namespace batch
