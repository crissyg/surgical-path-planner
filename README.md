# Surgical Path Planner

**3D Computational Geometry for Robotic-Assisted Surgery**

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/)
[![CMake](https://img.shields.io/badge/CMake-3.25%2B-green.svg)](https://cmake.org/)

## Overview

A production-ready C++20 application demonstrating path planning algorithms for robotic surgery systems like Stryker's Mako SmartRobotics. Features include:

- ✅ **3D Geometry**: Point and vector operations with double precision
- ✅ **Path Planning**: Optimal trajectory calculation with obstacle avoidance
- ✅ **Collision Detection**: Real-time safety validation
- ✅ **Modern C++20**: Smart pointers, constexpr, [[nodiscard]]
- ✅ **Clean Architecture**: SOLID principles, comprehensive documentation

## Quick Start

### Build (Linux/Mac)
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
./SurgicalPathPlanner
```

### Build (Windows)
```powershell
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022"
cmake --build . --config Release
.\Release\SurgicalPathPlanner.exe
```

## Relevance to Medical Device Software

This project demonstrates core concepts used in surgical robotics:
- **3D Spatial Planning** (like Mako's CT-based planning)
- **Real-time Safety Validation** (like Mako's AccuStop™ technology)
- **Computational Geometry** (for precise robotic positioning)

## Example Usage

```cpp
#include "surgical_path_planner/path/path_planner.hpp"

path::PathPlanner planner(3.0);  // 3mm safety margin
planner.addObstacle(Point3D(20, 25, 30), 5.0);  // Blood vessel

auto path = planner.planStraightPath(
    Point3D(10, 15, 0),   // Entry point
    Point3D(30, 40, 50)   // Target tumor
);

if (path.is_safe) {
    std::cout << "Safe path: " << path.total_length << " mm\n";
}
```

## Project Structure

```
surgical-path-planner/
├── CMakeLists.txt
├── README.md
├── include/surgical_path_planner/
│   ├── geometry/         # Point3D, Vector3D classes
│   ├── path/             # PathPlanner algorithm
│   ├── collision/        # Collision detection
│   └── visualization/    # Path visualization
└── src/                  # Implementation files
```

## Author

**Christina Gordon** 

## License

MIT License - See LICENSE file
