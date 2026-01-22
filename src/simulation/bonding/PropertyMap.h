#pragma once

#include <string>
#include <unordered_map>
#include <variant>
#include <optional>
#include <stdexcept>

namespace bonding
{

/// Value type for PropertyMap - supports common types
using PropertyValue = std::variant<bool, int, float, double, std::string>;

/// Generic property container for custom attributes
class PropertyMap
{
public:
    PropertyMap() = default;

    /// Set a property value
    template<typename T>
    void set(const std::string& key, T value)
    {
        m_properties[key] = PropertyValue(std::move(value));
    }

    /// Get a property value with type checking
    template<typename T>
    std::optional<T> get(const std::string& key) const
    {
        auto it = m_properties.find(key);
        if (it == m_properties.end())
            return std::nullopt;

        if (auto* val = std::get_if<T>(&it->second))
            return *val;

        return std::nullopt;
    }

    /// Get a property value with default
    template<typename T>
    T getOr(const std::string& key, T defaultValue) const
    {
        auto result = get<T>(key);
        return result.value_or(std::move(defaultValue));
    }

    /// Check if property exists
    bool has(const std::string& key) const
    {
        return m_properties.find(key) != m_properties.end();
    }

    /// Remove a property
    bool remove(const std::string& key)
    {
        return m_properties.erase(key) > 0;
    }

    /// Clear all properties
    void clear()
    {
        m_properties.clear();
    }

    /// Get number of properties
    size_t size() const
    {
        return m_properties.size();
    }

    /// Check if empty
    bool empty() const
    {
        return m_properties.empty();
    }

    /// Get underlying map for iteration
    const std::unordered_map<std::string, PropertyValue>& data() const
    {
        return m_properties;
    }

private:
    std::unordered_map<std::string, PropertyValue> m_properties;
};

} // namespace bonding
