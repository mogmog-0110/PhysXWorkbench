#include "CommandLineArgs.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <filesystem>

// Helper function for safe integer parsing
static bool SafeParseInt(const char* str, int& outValue, std::string& errorMsg)
{
    char* endptr = nullptr;
    errno = 0;
    long val = std::strtol(str, &endptr, 10);

    if (errno == ERANGE || val > INT_MAX || val < INT_MIN)
    {
        errorMsg = "Value out of range";
        return false;
    }
    if (endptr == str || *endptr != '\0')
    {
        errorMsg = "Invalid integer format";
        return false;
    }
    outValue = static_cast<int>(val);
    return true;
}

// Helper function for safe float parsing
static bool SafeParseFloat(const char* str, float& outValue, std::string& errorMsg)
{
    char* endptr = nullptr;
    errno = 0;
    float val = std::strtof(str, &endptr);

    if (errno == ERANGE)
    {
        errorMsg = "Value out of range";
        return false;
    }
    if (endptr == str || *endptr != '\0')
    {
        errorMsg = "Invalid float format";
        return false;
    }
    outValue = val;
    return true;
}

bool CommandLineArgs::Parse(int argc, char** argv)
{
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg == "--headless" || arg == "-h")
        {
            headless = true;
        }
        else if (arg == "--help")
        {
            help = true;
        }
        else if (arg == "--config" || arg == "-c")
        {
            if (i + 1 < argc)
            {
                configFile = argv[++i];
            }
            else
            {
                std::cerr << "Error: --config requires a filename" << std::endl;
                return false;
            }
        }
        else if (arg == "--output" || arg == "-o")
        {
            if (i + 1 < argc)
            {
                std::string rawPath = argv[++i];
                // Validate path doesn't contain directory traversal
                if (rawPath.find("..") != std::string::npos)
                {
                    std::cerr << "Error: --output path cannot contain '..'" << std::endl;
                    return false;
                }
                outputDir = rawPath;
                // Ensure trailing slash
                if (!outputDir.empty() && outputDir.back() != '/' && outputDir.back() != '\\')
                {
                    outputDir += "/";
                }
            }
            else
            {
                std::cerr << "Error: --output requires a directory path" << std::endl;
                return false;
            }
        }
        else if (arg == "--name" || arg == "-n")
        {
            if (i + 1 < argc)
            {
                std::string name = argv[++i];
                // Validate filename doesn't contain invalid characters
                const std::string invalidChars = "<>:\"/\\|?*";
                if (name.find_first_of(invalidChars) != std::string::npos)
                {
                    std::cerr << "Error: --name contains invalid filename characters" << std::endl;
                    return false;
                }
                experimentName = name;
            }
            else
            {
                std::cerr << "Error: --name requires an experiment name" << std::endl;
                return false;
            }
        }
        else if (arg == "--max-frames")
        {
            if (i + 1 < argc)
            {
                std::string parseError;
                if (!SafeParseInt(argv[++i], maxFrames, parseError))
                {
                    std::cerr << "Error: --max-frames: " << parseError << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "Error: --max-frames requires a number" << std::endl;
                return false;
            }
        }
        else if (arg == "--max-time")
        {
            if (i + 1 < argc)
            {
                std::string parseError;
                if (!SafeParseFloat(argv[++i], maxTime, parseError))
                {
                    std::cerr << "Error: --max-time: " << parseError << std::endl;
                    return false;
                }
            }
            else
            {
                std::cerr << "Error: --max-time requires a number" << std::endl;
                return false;
            }
        }
        else if (arg == "--record-interval")
        {
            if (i + 1 < argc)
            {
                std::string parseError;
                if (!SafeParseInt(argv[++i], recordInterval, parseError))
                {
                    std::cerr << "Error: --record-interval: " << parseError << std::endl;
                    return false;
                }
                if (recordInterval < 1) recordInterval = 1;
            }
            else
            {
                std::cerr << "Error: --record-interval requires a number" << std::endl;
                return false;
            }
        }
        else if (arg == "--seed")
        {
            if (i + 1 < argc)
            {
                int seedValue;
                std::string parseError;
                if (!SafeParseInt(argv[++i], seedValue, parseError))
                {
                    std::cerr << "Error: --seed: " << parseError << std::endl;
                    return false;
                }
                randomSeed = static_cast<unsigned int>(seedValue);
            }
            else
            {
                std::cerr << "Error: --seed requires a number" << std::endl;
                return false;
            }
        }
        else if (arg == "--scene")
        {
            if (i + 1 < argc)
            {
                sceneType = argv[++i];
            }
            else
            {
                std::cerr << "Error: --scene requires a scene name" << std::endl;
                return false;
            }
        }
        else if (arg == "--verbose" || arg == "-v")
        {
            verbose = true;
        }
        else if (arg == "--no-profiling")
        {
            enableProfiling = false;
        }
        else
        {
            std::cerr << "Warning: Unknown argument '" << arg << "'" << std::endl;
        }
    }

    return true;
}

void CommandLineArgs::PrintHelp()
{
    std::cout << "PhysX Research Experiment Tool\n";
    std::cout << "================================\n\n";
    std::cout << "Usage: PhysXSampleDX12.exe [options]\n\n";
    std::cout << "Execution Mode:\n";
    std::cout << "  --headless, -h          Run without rendering (10-100x faster)\n";
    std::cout << "  --help                  Show this help message\n\n";

    std::cout << "Input/Output:\n";
    std::cout << "  --config, -c <file>     JSON configuration file for experiment\n";
    std::cout << "  --output, -o <dir>      Output directory (default: results/)\n";
    std::cout << "  --name, -n <name>       Experiment name for output files\n\n";

    std::cout << "Simulation Control:\n";
    std::cout << "  --max-frames <N>        Stop after N frames (-1 = unlimited)\n";
    std::cout << "  --max-time <T>          Stop after T seconds (-1 = unlimited)\n";
    std::cout << "  --record-interval <N>   Record every N frames (default: 1)\n";
    std::cout << "  --seed <N>              Random seed for deterministic runs (0 = use time)\n";
    std::cout << "  --scene <name>          Scene type (basic_stack, stress_test, etc.)\n\n";

    std::cout << "Performance:\n";
    std::cout << "  --verbose, -v           Verbose output\n";
    std::cout << "  --no-profiling          Disable performance profiling\n\n";

    std::cout << "Examples:\n";
    std::cout << "  # Interactive mode with GUI\n";
    std::cout << "  PhysXSampleDX12.exe\n\n";

    std::cout << "  # Headless mode for batch experiments\n";
    std::cout << "  PhysXSampleDX12.exe --headless --config experiment.json --output results/\n\n";

    std::cout << "  # Quick test simulation\n";
    std::cout << "  PhysXSampleDX12.exe --headless --scene stress_test --max-frames 1000\n\n";

    std::cout << "  # Deterministic run for reproducibility\n";
    std::cout << "  PhysXSampleDX12.exe --headless --seed 42 --max-time 10.0\n\n";
}

bool CommandLineArgs::Validate(std::string& errorMessage)
{
    // If headless mode, we need a way to stop the simulation
    if (headless && maxFrames < 0 && maxTime < 0.0f && configFile.empty())
    {
        errorMessage = "Headless mode requires --max-frames, --max-time, or --config to specify when to stop";
        return false;
    }

    // Record interval must be positive
    if (recordInterval < 1)
    {
        errorMessage = "Record interval must be at least 1";
        return false;
    }

    return true;
}
