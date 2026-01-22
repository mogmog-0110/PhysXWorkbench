# Scene Definition Format

This directory contains JSON scene definitions that can be loaded into the PhysX sample application.

## JSON Scene Format

### Top-Level Structure

```json
{
  "name": "Scene Name",
  "description": "Description of the scene",
  "gravity": [0, -9.81, 0],
  "actors": [ ... ],
  "joints": [ ... ]
}
```

### Actor Definition

#### Static Plane
```json
{
  "name": "ground",
  "type": "static",
  "geometry": "plane",
  "position": [0, 0, 0]
}
```

#### Dynamic Box
```json
{
  "name": "box1",
  "type": "dynamic",
  "geometry": "box",
  "position": [0, 10, 0],
  "halfExtents": [1.0, 0.5, 0.5],
  "density": 1.0,
  "friction": 0.5,
  "restitution": 0.3
}
```

#### Dynamic Sphere
```json
{
  "name": "sphere1",
  "type": "dynamic",
  "geometry": "sphere",
  "position": [0, 5, 5],
  "radius": 0.5,
  "density": 1.0,
  "friction": 0.5,
  "restitution": 0.6
}
```

### Actor Properties

- **name**: Unique identifier for the actor (used for joints)
- **type**: "static" or "dynamic"
- **geometry**: "box", "sphere", or "plane"
- **position**: [x, y, z] - World position
- **halfExtents**: [x, y, z] - Box half-extents (for box geometry)
- **radius**: Sphere radius (for sphere geometry)
- **density**: Mass density (kg/m³) for dynamic actors
- **friction**: Friction coefficient (0.0 - 1.0)
- **restitution**: Bounciness (0.0 = no bounce, 1.0 = perfect bounce)

### Joint Definition

#### Fixed Joint
```json
{
  "name": "joint1",
  "type": "fixed",
  "actor1": "box1",
  "actor2": "box2",
  "breakable": true,
  "breakForce": 1000.0
}
```

#### Spherical Joint (Ball-and-Socket)
```json
{
  "name": "joint2",
  "type": "spherical",
  "actor1": "box2",
  "actor2": "box3",
  "limitEnabled": true
}
```

#### D6 Joint (6 Degrees of Freedom)
```json
{
  "name": "joint3",
  "type": "d6",
  "actor1": "box3",
  "actor2": "box4"
}
```

### Joint Properties

- **name**: Unique identifier for the joint
- **type**: "fixed", "spherical", "revolute", "prismatic", "distance", or "d6"
- **actor1**: Name of first actor
- **actor2**: Name of second actor (empty string for world attachment)
- **breakable**: true/false - Whether joint can break
- **breakForce**: Force threshold for breaking (Newtons)
- **limitEnabled**: true/false - Enable joint limits

## Usage

### Loading Scenes Programmatically

```cpp
#include "simulation/SceneLoader.h"

SceneLoader loader;
SceneDefinition sceneDef;

// Load scene definition from JSON
if (loader.LoadFromJSON("scenes/example_scene.json", sceneDef))
{
    std::vector<PxRigidActor*> actors;
    std::vector<PxJoint*> joints;

    // Create scene from definition
    if (loader.CreateScene(gPhysics, sceneDef, gScene, gMaterial, actors, joints))
    {
        std::cout << "Scene loaded successfully!" << std::endl;
    }
    else
    {
        std::cerr << "Failed to create scene: " << loader.GetLastError() << std::endl;
    }
}
else
{
    std::cerr << "Failed to load scene: " << loader.GetLastError() << std::endl;
}
```

## Example Scenes

- **example_scene.json**: Basic scene with boxes and spheres connected by joints

## Creating Custom Scenes

1. Copy `example_scene.json` to a new file
2. Modify the actors and joints as needed
3. Save the file in the `scenes/` directory
4. Load it using the SceneLoader class

## Supported Features

- ✅ Box geometry
- ✅ Sphere geometry
- ✅ Plane geometry (static only)
- ✅ Static and dynamic actors
- ✅ Fixed joints
- ✅ Spherical joints
- ✅ D6 joints
- ✅ Breakable joints
- ✅ Joint limits
- ⚠️ Revolute joints (defined but not fully tested)
- ⚠️ Prismatic joints (defined but not fully tested)
- ⚠️ Distance joints (defined but not fully tested)

## Limitations

- The JSON parser is basic and may not handle all edge cases
- Complex joint configurations may require code-level setup
- For production use, consider integrating a proper JSON library (e.g., nlohmann/json)
