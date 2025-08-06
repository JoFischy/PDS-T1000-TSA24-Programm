# PDS-T1000-TSA24-Programm

## Vehicle Path Following System

This program demonstrates a vehicle path following system with 4 fixed vehicles that automatically follow a rectangular path.

### Features

- **4 Fixed Vehicles**: Vehicles are positioned in a square formation and move along a predefined path
- **Automatic Path Following**: Vehicles follow a rectangular path using pathfinding logic
- **Real-time Controls**: Speed and movement controls for all vehicles
- **Path Visualization**: Optional display of the movement path

### Controls

| Key | Action |
|-----|--------|
| `ESC` | Exit program |
| `P` | Toggle path display on/off |
| `R` | Recreate sample path |
| `↑` | Increase all vehicle speeds to 3.0 |
| `↓` | Decrease all vehicle speeds to 1.0 |
| `SPACE` | Start/Stop all vehicles |

### Building

Use the provided build script:

```bash
# Windows
.\build.bat

# Or manually with g++
g++ -D_USE_MATH_DEFINES -DPLATFORM_DESKTOP -Iinclude -Iexternal/raylib -std=c++17 -O2 src/*.cpp -o PDS-T1000.exe -Lexternal/raylib/lib -lraylib -lopengl32 -lgdi32 -lwinmm
```

### Changes from Previous Version

- **Removed**: Draggable points system
- **Added**: 4 fixed vehicles with center points and directions
- **Enhanced**: Automatic movement based on path following logic
- **Improved**: Vehicle state management with start/stop functionality

### Architecture

- `PointManager`: Main application class managing vehicles and rendering
- `VehicleController`: Handles vehicle movement and path following
- `PathSystem`: Manages the rectangular path nodes
- `Auto`: Vehicle representation with position and direction
- `Renderer`: Graphics rendering with Raylib