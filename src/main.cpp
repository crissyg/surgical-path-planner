#include "surgical_path_planner/geometry/point3d.hpp"
#include "surgical_path_planner/geometry/vector3d.hpp"
#include "surgical_path_planner/path/path_planner.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace surgical_path_planner;
using Point = geometry::Point3D;
using Vector = geometry::Vector3D;

void printSectionHeader(const std::string& title) {
    std::cout << "\n========================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "========================================\n";
}

void printPathDetails(const path::SurgicalPath& path, const std::string& label) {
    std::cout << "\n" << label << ":\n";
    std::cout << "  Entry:  " << path.entry_point.toString() << "\n";
    std::cout << "  Target: " << path.target_point.toString() << "\n";
    std::cout << "  Length: " << std::fixed << std::setprecision(2) << path.total_length << " mm\n";
    std::cout << "  Angle:  " << (path.approach_angle * 180.0 / M_PI) << "°\n";
    std::cout << "  Safe:   " << (path.is_safe ? "✓ YES" : "✗ NO") << "\n";
}

int main() {
    std::cout << "\n╔═══════════════════════════════════════════════╗\n";
    std::cout << "║      SURGICAL PATH PLANNER v1.0.0             ║\n";
    std::cout << "║   3D Computational Geometry for Robotics      ║\n";
    std::cout << "╚═══════════════════════════════════════════════╝\n";

    printSectionHeader("Demo 1: 3D Geometry Operations");
    Point entry(0, 0, 0);
    Point tumor(30, 40, 50);
    std::cout << "\nEntry:  " << entry.toString();
    std::cout << "\nTumor:  " << tumor.toString();
    std::cout << "\nDistance: " << std::fixed << std::setprecision(2) 
              << entry.distanceTo(tumor) << " mm\n";

    Vector direction = Vector::fromPoints(entry, tumor);
    std::cout << "Direction: " << direction.toString();
    std::cout << "\nMagnitude: " << direction.magnitude() << " mm\n";

    printSectionHeader("Demo 2: Straight Path Planning");
    path::PathPlanner planner(3.0);
    auto path1 = planner.planStraightPath(Point(10, 15, 0), Point(10, 15, 60));
    printPathDetails(path1, "Simple Path");

    printSectionHeader("Demo 3: Obstacle Avoidance");
    planner.addObstacle(Point(25, 30, 35), 5.0);
    planner.addObstacle(Point(22, 28, 45), 3.0);
    std::cout << "\nObstacles: " << planner.getObstacleCount() << "\n";

    auto path2 = planner.planStraightPath(Point(20, 25, 0), Point(25, 30, 50));
    printPathDetails(path2, "Obstructed Path");

    if (!path2.is_safe) {
        auto alternatives = planner.findAlternativePaths(Point(25, 30, 50), 25.0, 8);
        std::cout << "\nAlternatives found: " << alternatives.size() << "\n";
        if (!alternatives.empty()) {
            printPathDetails(alternatives[0], "Best Alternative");
        }
    }

    std::cout << "\n✓ All demonstrations complete!\n\n";
    return 0;
}
