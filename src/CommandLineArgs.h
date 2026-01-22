#pragma once

#include <string>
#include <vector>

// Command line arguments structure for research experiments
struct CommandLineArgs
{
    // Execution mode
    bool headless = false;           // Run without rendering (10-100x faster)
    bool help = false;               // Show help message

    // Input/Output
    std::string configFile;          // JSON configuration file for experiment
    std::string outputDir = "results/";  // Output directory for results
    std::string experimentName;      // Name of this experiment run

    // Simulation control
    int maxFrames = -1;              // Maximum frames to simulate (-1 = unlimited)
    float maxTime = -1.0f;           // Maximum simulation time (-1 = unlimited)
    int recordInterval = 1;          // Record every N frames (1 = every frame)

    // Performance
    bool enableProfiling = true;     // Enable performance profiling
    bool verbose = false;            // Verbose output

    // Random seed for deterministic simulation
    unsigned int randomSeed = 0;     // 0 = use current time

    // Scene selection (if not using config file)
    std::string sceneType = "basic_stack";

    // Parse command line arguments
    bool Parse(int argc, char** argv);

    // Display help message
    void PrintHelp();

    // Validate arguments
    bool Validate(std::string& errorMessage);
};
