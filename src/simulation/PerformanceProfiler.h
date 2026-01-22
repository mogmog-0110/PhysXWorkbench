#pragma once

#include <windows.h>
#include <vector>
#include <string>
#include <deque>

class PerformanceProfiler
{
public:
    PerformanceProfiler();
    ~PerformanceProfiler();

    // Frame timing
    void BeginFrame();
    void EndFrame();

    void BeginPhysics();
    void EndPhysics();

    void BeginRendering();
    void EndRendering();

    // Get timing statistics
    float GetAverageFrameTime() const;
    float GetAveragePhysicsTime() const;
    float GetAverageRenderTime() const;

    float GetCurrentFrameTime() const { return m_lastFrameTime; }
    float GetCurrentPhysicsTime() const { return m_lastPhysicsTime; }
    float GetCurrentRenderTime() const { return m_lastRenderTime; }

    float GetAverageFPS() const;
    float GetCurrentFPS() const;

    // Get history for graphing
    const std::deque<float>& GetFrameTimeHistory() const { return m_frameTimeHistory; }
    const std::deque<float>& GetPhysicsTimeHistory() const { return m_physicsTimeHistory; }
    const std::deque<float>& GetRenderTimeHistory() const { return m_renderTimeHistory; }

    // Configuration
    void SetHistorySize(size_t size) { m_historySize = size; }
    size_t GetHistorySize() const { return m_historySize; }

    // Export profiling data
    bool ExportToCSV(const std::string& filename);

    // Reset statistics
    void Reset();

private:
    LARGE_INTEGER m_frequency;

    LARGE_INTEGER m_frameStart;
    LARGE_INTEGER m_frameEnd;

    LARGE_INTEGER m_physicsStart;
    LARGE_INTEGER m_physicsEnd;

    LARGE_INTEGER m_renderStart;
    LARGE_INTEGER m_renderEnd;

    float m_lastFrameTime;
    float m_lastPhysicsTime;
    float m_lastRenderTime;

    std::deque<float> m_frameTimeHistory;
    std::deque<float> m_physicsTimeHistory;
    std::deque<float> m_renderTimeHistory;

    size_t m_historySize;

    // Helper functions
    float GetElapsedTime(LARGE_INTEGER start, LARGE_INTEGER end) const;
    void AddToHistory(std::deque<float>& history, float value);
    float GetAverage(const std::deque<float>& history) const;
};
