# PDS-T1000-TSA24-Programm

## Vehicle Path Following System

This program demonstrates a vehicle path following system that automatically detects white paths in a factory layout image and places 4 vehicles to follow the detected path network.

### Features

- **Automatic Path Detection**: Detects white pathways in the factory layout image
- **4 Fixed Vehicles**: Automatically spawns 4 vehicles at different intersections
- **Smart Node Placement**: Places path nodes at detected intersections of white stripes
- **Automatic Path Following**: Vehicles follow paths between detected intersections
- **Real-time Path Visualization**: Display of detected path network and connections
- **Dynamic Target Assignment**: Vehicles automatically get new random targets when reaching destinations

### Controls

| Key | Action |
|-----|--------|
| `ESC` | Exit program |
| `1` | Toggle path nodes display on/off |
| `2` | Toggle path segments display on/off |
| `3` | Toggle intersections display on/off |
| `4` | Toggle vehicle IDs display on/off |
| `5` | Toggle debug info display on/off |
| `SPACE` | Pause/Resume simulation |
| `V` | Manually spawn additional vehicle |
| `WASD` | Move camera |
| `Mouse Wheel` | Zoom in/out |
| `R` | Reset camera position |

### Building

Use the provided build script:

```bash
# Windows
.\build.bat

# Or manually with g++
g++ -D_USE_MATH_DEFINES -DPLATFORM_DESKTOP -Iinclude -Iexternal/raylib -std=c++17 -O2 src/*.cpp -o PDS-T1000.exe -Lexternal/raylib/lib -lraylib -lopengl32 -lgdi32 -lwinmm
```

### Changes from Previous Version

- **Removed**: Manual draggable points system
- **Added**: Automatic image-based path detection from `factory_layout.png`
- **Added**: Smart intersection detection at white stripe crossings  
- **Enhanced**: 4 vehicles automatically placed at different detected intersections
- **Enhanced**: Dynamic target assignment system
- **Improved**: Path visualization showing detected nodes and connections
- **Added**: Camera controls for navigation and zoom

### Architecture

- `Simulation`: Main application class managing the simulation loop
- `PathDetector`: Analyzes factory layout image to detect white paths and intersections
- `PathSystem`: Manages the detected path network with nodes and segments
- `VehicleController`: Handles 4 vehicles, movement, and target assignment
- `SegmentManager`: Manages traffic flow and collision avoidance
- `Auto`: Vehicle representation with position, direction, and pathfinding
- `Renderer`: Graphics rendering with Raylib, including path visualization