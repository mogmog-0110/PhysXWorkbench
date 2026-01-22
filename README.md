# PhysXWorkbench

A physics simulation research platform built on NVIDIA PhysX 5.x with DirectX 12 real-time rendering.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey.svg)
![DirectX](https://img.shields.io/badge/DirectX-12-green.svg)
![PhysX](https://img.shields.io/badge/PhysX-5.6.1-76B900.svg)

## Features

### Core Simulation
- **PhysX 5.6.1 Integration** - Full support for rigid body dynamics, collision detection, and constraints
- **GPU PhysX Support** - CUDA acceleration with automatic CPU fallback
- **Real-time DirectX 12 Rendering** - GPU-accelerated visualization with ImGui interface
- **Scene Management** - JSON-based scene definition and loading

### Dynamic Bonding System
A flexible system for creating and managing dynamic bonds between physics entities:

- **Bonding Sites** - Define attachment points with position, direction, and valency
- **Bond Formation Rules** - Pluggable rules for automatic bond creation:
  - Proximity-based detection
  - Directional alignment
  - Valency constraints
  - Coplanarity requirements
  - Angle constraints
- **Bond Types** - Multiple joint types supported:
  - Rigid (Fixed joints)
  - Compliant (Spring-damper)
  - Hinged (Revolute)
  - Ball-socket (Spherical)
  - Prismatic (Linear)
  - D6 (Fully configurable)

### Batch Simulation System
Tools for running multiple simulations and collecting statistics:

- **Metrics Collection** - Track bond counts, kinetic energy, cluster sizes, ring formation
- **Termination Conditions** - Timeout, steady-state detection (MSER), target conditions
- **Data Export** - CSV and JSON output for analysis
- **Replication** - Run multiple simulations with different seeds

### Rendering
- DirectX 12 rendering pipeline
- Blinn-Phong lighting model
- Real-time camera controls (WASD + mouse)
- Depth buffering and perspective projection
- Mesh caching system for performance

### Debug UI (ImGui)
- FPS and frame time monitoring
- PhysX statistics (actor counts, gravity control)
- Camera position tracking
- Mesh cache statistics
- GPU PhysX status indicator

## Requirements

- **Windows 10/11** (64-bit)
- **Visual Studio 2022** (v143 toolset)
- **CMake 3.16** or later
- **NVIDIA PhysX SDK 5.6.1**
- **DirectX 12 capable GPU**
- **CUDA-capable NVIDIA GPU** (optional, for GPU PhysX acceleration)

## Building

### 1. Clone PhysX SDK

```bash
git clone https://github.com/NVIDIA-Omniverse/PhysX.git
cd PhysX
git checkout 5.6.1
```

### 2. Build PhysX

Windows (Visual Studio 2022):
```bash
cd physx
./generate_projects.bat vc17win64
```

Open `physx/compiler/vc17win64/PhysXSDK.sln` in Visual Studio and build the **Debug** configuration.

### 3. Clone This Repository

```bash
cd ..
git clone https://github.com/YOUR_USERNAME/PhysXWorkbench.git
cd PhysXWorkbench
```

### 4. Configure and Build

```bash
mkdir build
cd build
cmake -DPHYSX_ROOT_DIR=../../PhysX/physx ..
cmake --build . --config Debug
```

### 5. Run

```bash
cd Debug
PhysXWorkbench.exe
```

## Executables

| Executable | Description |
|------------|-------------|
| `PhysXWorkbench.exe` | Main application with DX12 rendering and full features |
| `PhysXWorkbenchConsole.exe` | Console-only version for headless simulation |

## Controls

| Key/Mouse | Action |
|-----------|--------|
| **W/A/S/D** | Move camera forward/left/backward/right |
| **Q/E** | Move camera up/down |
| **Right Mouse Button + Drag** | Look around (FPS-style camera) |
| **ESC** | Exit application |

## Project Structure

```
PhysXWorkbench/
├── src/
│   ├── main_dx12.cpp              # Main application entry point
│   ├── dx12/                      # DirectX 12 renderer
│   │   ├── DX12Renderer.h/cpp
│   │   └── d3dx12.h
│   ├── simulation/
│   │   ├── bonding/               # Dynamic bonding system
│   │   │   ├── BondableEntity.h/cpp
│   │   │   ├── DynamicBondManager.h/cpp
│   │   │   ├── BondFormationRules.h/cpp
│   │   │   └── BondTypes.h/cpp
│   │   ├── batch/                 # Batch simulation system
│   │   │   ├── BatchSimulationRunner.h/cpp
│   │   │   ├── CommonMetrics.h/cpp
│   │   │   └── CommonTerminationConditions.h/cpp
│   │   ├── SceneLoader.h/cpp
│   │   └── SimulationRecorder.h/cpp
│   └── CommandLineArgs.h/cpp
├── shaders/
│   ├── PhysicsVS.hlsl             # Vertex shader
│   └── PhysicsPS.hlsl             # Pixel shader (Blinn-Phong)
├── external/
│   └── imgui/                     # Dear ImGui library
├── CMakeLists.txt
└── README.md
```

## Usage Examples

### Basic Usage

```bash
# Run the main application
./Debug/PhysXWorkbench.exe

# Run with specific scene
./Debug/PhysXWorkbench.exe --scene path/to/scene.json
```

### Batch Simulation (C++ API)

```cpp
#include "simulation/batch/BatchSimulationRunner.h"

batch::BatchConfig config;
config.numReplicates = 100;
config.maxSimulationTime = 60.0f;

batch::BatchSimulationRunner runner;
runner.configure(config);
runner.addMetric(std::make_shared<batch::BondCountMetric>(bondManager));
runner.addTerminationCondition(std::make_shared<batch::SteadyStateCondition>(0.01f));

auto results = runner.run(physics, sceneFactory);
runner.exportToCSV(results, "results.csv");
```

### Dynamic Bonding (C++ API)

```cpp
#include "simulation/bonding/BondingIntegration.h"

// Create bond manager
auto bondManager = std::make_unique<bonding::DynamicBondManager>();
bondManager->initialize(physics, scene);

// Add formation rules
bondManager->addRule(std::make_unique<bonding::ProximityRule>(1.0f));
bondManager->addRule(std::make_unique<bonding::DirectionalAlignmentRule>(0.8f));
bondManager->addRule(std::make_unique<bonding::ValencyRule>());

// Register bond type
bondManager->registerBondType(std::make_unique<bonding::CompliantBondType>(1000.0f, 10.0f));

// Update each frame
bondManager->update(deltaTime);
```

## Troubleshooting

### GPU PhysX Shows "DISABLED"
- Ensure you have a CUDA-capable NVIDIA GPU
- PhysX must be built with GPU support enabled
- Required DLLs: PhysXGpu_64.dll, PhysXDevice64.dll

### Application Crashes on Startup
- Verify PhysX DLLs are in the executable directory
- Check shader files are in ../shaders/ relative to executable
- Ensure DirectX 12 drivers are up to date

## License

MIT License - See LICENSE file for details

## Acknowledgments

- **NVIDIA PhysX** - Physics simulation engine
- **Dear ImGui** - Immediate mode GUI library
- **Microsoft DirectX 12** - Graphics API

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.
