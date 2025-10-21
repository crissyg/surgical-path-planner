#pragma once

#include "surgical_path_planner/geometry/point3d.hpp"
#include "surgical_path_planner/geometry/vector3d.hpp"
#include <vector>
#include <string>

namespace surgical_path_planner {
namespace path {

struct SurgicalPath {
    geometry::Point3D entry_point;
    geometry::Point3D target_point;
    std::vector<geometry::Point3D> waypoints;
    double total_length;
    double approach_angle;
    bool is_safe;
    std::string safety_notes;

    SurgicalPath() : total_length(0.0), approach_angle(0.0), is_safe(false) {}
};

class PathPlanner {
public:
    explicit PathPlanner(double min_clearance = 3.0);
    ~PathPlanner() = default;

    void addObstacle(const geometry::Point3D& center, double radius);
    void clearObstacles() noexcept;

    [[nodiscard]] SurgicalPath planStraightPath(
        const geometry::Point3D& entry_point,
        const geometry::Point3D& target_point
    ) const;

    [[nodiscard]] std::vector<SurgicalPath> findAlternativePaths(
        const geometry::Point3D& target_point,
        double search_radius,
        int num_candidates = 8
    ) const;

    [[nodiscard]] double getMinClearance() const noexcept { return min_clearance_; }
    [[nodiscard]] std::size_t getObstacleCount() const noexcept;

private:
    struct Obstacle {
        geometry::Point3D center;
        double radius;
        Obstacle(const geometry::Point3D& c, double r) : center(c), radius(r) {}
    };

    std::vector<Obstacle> obstacles_;
    double min_clearance_;

    [[nodiscard]] bool isPathClear(
        const geometry::Point3D& start,
        const geometry::Point3D& end
    ) const noexcept;

    [[nodiscard]] static double pointToLineDistance(
        const geometry::Point3D& point,
        const geometry::Point3D& line_start,
        const geometry::Point3D& line_end
    ) noexcept;
};

} // namespace path
} // namespace surgical_path_planner
