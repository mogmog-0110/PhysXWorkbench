# PhysX DirectX 12 Rendering Tool

A real-time physics simulation visualization tool built with NVIDIA PhysX 5.6.1 and DirectX 12.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-Windows-lightgrey.svg)
![DirectX](https://img.shields.io/badge/DirectX-12-green.svg)

## Features

### Phase 1: Foundation (Completed ✅)
- **Mesh Caching System**: Intelligent geometry caching for reduced GPU resource creation overhead
- **ImGui Debug UI**: Real-time debug interface with:
  - FPS and frame time monitoring
  - PhysX statistics (actor counts, gravity control)
  - Camera position tracking
  - Mesh cache statistics
  - GPU PhysX status indicator
- **GPU PhysX Support**: CUDA acceleration support (when available)
  - Automatic fallback to CPU if GPU not available
  - Real-time status display in debug UI

### Rendering
- DirectX 12 rendering pipeline
- Blinn-Phong lighting model
- Real-time camera controls (WASD + mouse)
- Depth buffering and perspective projection

### Physics Simulation
- NVIDIA PhysX 5.6.1 integration
- Rigid body dynamics (boxes, spheres)
- Collision detection
- Ground plane with realistic friction
- Configurable gravity

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
git clone https://github.com/YOUR_USERNAME/PhysXSample.git
cd PhysXSample
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
PhysXSampleDX12.exe
```

## Controls

| Key/Mouse | Action |
|-----------|--------|
| **W/A/S/D** | Move camera forward/left/backward/right |
| **Q/E** | Move camera up/down |
| **Right Mouse Button + Drag** | Look around (FPS-style camera) |
| **ESC** | Exit application |

### ImGui Interface
- **Gravity Slider**: Adjust gravity in real-time (Y-axis: -20.0 to 0.0)
- **Statistics**: Monitor actors, FPS, and cache efficiency

## Project Structure

```
PhysXSample/
├── src/
│   ├── main_dx12.cpp           # Application entry point
│   └── dx12/
│       ├── DX12Renderer.h      # DirectX 12 renderer interface
│       ├── DX12Renderer.cpp    # Renderer implementation
│       └── d3dx12.h            # DirectX 12 helpers
├── shaders/
│   ├── PhysicsVS.hlsl          # Vertex shader (MVP transform)
│   ├── PhysicsPS.hlsl          # Pixel shader (Blinn-Phong)
│   ├── PBR_VS.hlsl             # PBR vertex shader (WIP)
│   └── PBR_PS.hlsl             # PBR pixel shader (WIP)
├── external/
│   └── imgui/                  # Dear ImGui library
├── CMakeLists.txt              # Build configuration
└── README.md
```

## Architecture

### DirectX 12 Pipeline
1. **Device & Command Queue**: Low-level GPU control
2. **Swap Chain**: Double-buffered rendering
3. **Descriptor Heaps**: Resource binding (RTV, DSV, CBV, SRV for ImGui)
4. **Pipeline State**: Vertex/pixel shaders, rasterizer, blend state
5. **Root Signature**: Shader parameter layout
6. **Mesh Caching**: Reusable geometry resources

### PhysX Integration
- Foundation, Physics, Scene management
- CPU Dispatcher (multi-threaded simulation)
- CUDA Context Manager (optional GPU acceleration)
- Material properties (friction, restitution)

## Troubleshooting

### GPU PhysX Shows "DISABLED"
- Ensure you have a CUDA-capable NVIDIA GPU
- PhysX must be built with GPU support enabled
- Required DLLs: PhysXGpu_64.dll, PhysXDevice64.dll
- These are generated when PhysX is built with PX_GENERATE_GPU_PROJECTS=1

### Application Crashes on Startup
- Verify PhysX DLLs are in the executable directory
- Check shader files are in ../shaders/ relative to executable
- Ensure DirectX 12 drivers are up to date

### ImGui Not Responding to Input
- Fixed in latest version with WantCaptureKeyboard/WantCaptureMouse checks
- Rebuild the project

## License

MIT License - See LICENSE file for details

## Acknowledgments

- **NVIDIA PhysX**: Physics simulation engine
- **Dear ImGui**: Immediate mode GUI library
- **Microsoft DirectX 12**: Graphics API

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.
