#include "PerformanceProfiler.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>

PerformanceProfiler::PerformanceProfiler()
    : m_lastFrameTime(0.0f)
    , m_lastPhysicsTime(0.0f)
    , m_lastRenderTime(0.0f)
    , m_historySize(300)  // Default: 5 seconds at 60 FPS
{
    QueryPerformanceFrequency(&m_frequency);
}

PerformanceProfiler::~PerformanceProfiler()
{
}

void PerformanceProfiler::BeginFrame()
{
    QueryPerformanceCounter(&m_frameStart);
}

void PerformanceProfiler::EndFrame()
{
    QueryPerformanceCounter(&m_frameEnd);
    m_lastFrameTime = GetElapsedTime(m_frameStart, m_frameEnd);
    AddToHistory(m_frameTimeHistory, m_lastFrameTime);
}

void PerformanceProfiler::BeginPhysics()
{
    QueryPerformanceCounter(&m_physicsStart);
}

void PerformanceProfiler::EndPhysics()
{
    QueryPerformanceCounter(&m_physicsEnd);
    m_lastPhysicsTime = GetElapsedTime(m_physicsStart, m_physicsEnd);
    AddToHistory(m_physicsTimeHistory, m_lastPhysicsTime);
}

void PerformanceProfiler::BeginRendering()
{
    QueryPerformanceCounter(&m_renderStart);
}

void PerformanceProfiler::EndRendering()
{
    QueryPerformanceCounter(&m_renderEnd);
    m_lastRenderTime = GetElapsedTime(m_renderStart, m_renderEnd);
    AddToHistory(m_renderTimeHistory, m_lastRenderTime);
}

float PerformanceProfiler::GetAverageFrameTime() const
{
    return GetAverage(m_frameTimeHistory);
}

float PerformanceProfiler::GetAveragePhysicsTime() const
{
    return GetAverage(m_physicsTimeHistory);
}

float PerformanceProfiler::GetAverageRenderTime() const
{
    return GetAverage(m_renderTimeHistory);
}

float PerformanceProfiler::GetAverageFPS() const
{
    float avgFrameTime = GetAverageFrameTime();
    return (avgFrameTime > 0.0f) ? (1000.0f / avgFrameTime) : 0.0f;
}

float PerformanceProfiler::GetCurrentFPS() const
{
    return (m_lastFrameTime > 0.0f) ? (1000.0f / m_lastFrameTime) : 0.0f;
}

bool PerformanceProfiler::ExportToCSV(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    file << "Frame,FrameTime(ms),PhysicsTime(ms),RenderTime(ms),FPS\n";

    size_t numSamples = m_frameTimeHistory.size();
    for (size_t i = 0; i < numSamples; i++)
    {
        float frameTime = (i < m_frameTimeHistory.size()) ? m_frameTimeHistory[i] : 0.0f;
        float physicsTime = (i < m_physicsTimeHistory.size()) ? m_physicsTimeHistory[i] : 0.0f;
        float renderTime = (i < m_renderTimeHistory.size()) ? m_renderTimeHistory[i] : 0.0f;
        float fps = (frameTime > 0.0f) ? (1000.0f / frameTime) : 0.0f;

        file << i << ","
             << std::fixed << std::setprecision(3)
             << frameTime << ","
             << physicsTime << ","
             << renderTime << ","
             << fps << "\n";
    }

    file.close();
    std::cout << "Exported " << numSamples << " performance samples to " << filename << std::endl;
    return true;
}

void PerformanceProfiler::Reset()
{
    m_frameTimeHistory.clear();
    m_physicsTimeHistory.clear();
    m_renderTimeHistory.clear();
    m_lastFrameTime = 0.0f;
    m_lastPhysicsTime = 0.0f;
    m_lastRenderTime = 0.0f;
}

float PerformanceProfiler::GetElapsedTime(LARGE_INTEGER start, LARGE_INTEGER end) const
{
    // Returns time in milliseconds
    return static_cast<float>(end.QuadPart - start.QuadPart) * 1000.0f / m_frequency.QuadPart;
}

void PerformanceProfiler::AddToHistory(std::deque<float>& history, float value)
{
    history.push_back(value);
    while (history.size() > m_historySize)
    {
        history.pop_front();
    }
}

float PerformanceProfiler::GetAverage(const std::deque<float>& history) const
{
    if (history.empty())
        return 0.0f;

    float sum = std::accumulate(history.begin(), history.end(), 0.0f);
    return sum / history.size();
}
