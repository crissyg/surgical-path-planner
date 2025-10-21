#include "surgical_path_planner/path/path_planner.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace surgical_path_planner {
namespace path {

PathPlanner::PathPlanner(double min_clearance) : min_clearance_(min_clearance) {
    if (min_clearance <= 0.0) {
        throw std::invalid_argument("Minimum clearance must be positive");
    }
}

void PathPlanner::addObstacle(const geometry::Point3D& center, double radius) {
    if (radius <= 0.0) {
        throw std::invalid_argument("Obstacle radius must be positive");
    }
    obstacles_.emplace_back(center, radius);
}

void PathPlanner::clearObstacles() noexcept {
    obstacles_.clear();
}

std::size_t PathPlanner::getObstacleCount() const noexcept {
    return obstacles_.size();
}

double PathPlanner::pointToLineDistance(
    const geometry::Point3D& point,
    const geometry::Point3D& line_start,
    const geometry::Point3D& line_end
) noexcept {
    geometry::Vector3D line_vec = geometry::Vector3D::fromPoints(line_start, line_end);
    geometry::Vector3D point_vec = geometry::Vector3D::fromPoints(line_start, point);

    const double line_length = line_vec.magnitude();
    if (line_length == 0.0) {
        return point_vec.magnitude();
    }

    const double t = std::max(0.0, std::min(1.0, point_vec.dot(line_vec) / (line_length * line_length)));

    geometry::Point3D closest(
        line_start.x() + t * line_vec.x(),
        line_start.y() + t * line_vec.y(),
        line_start.z() + t * line_vec.z()
    );

    return point.distanceTo(closest);
}

bool PathPlanner::isPathClear(
    const geometry::Point3D& start,
    const geometry::Point3D& end
) const noexcept {
    for (const auto& obstacle : obstacles_) {
        const double distance = pointToLineDistance(obstacle.center, start, end);
        const double required_distance = obstacle.radius + min_clearance_;

        if (distance < required_distance) {
            return false;
        }
    }
    return true;
}

SurgicalPath PathPlanner::planStraightPath(
    const geometry::Point3D& entry_point,
    const geometry::Point3D& target_point
) const {
    SurgicalPath path;
    path.entry_point = entry_point;
    path.target_point = target_point;
    path.total_length = entry_point.distanceTo(target_point);

    geometry::Vector3D direction = geometry::Vector3D::fromPoints(entry_point, target_point);
    geometry::Vector3D vertical(0.0, 0.0, 1.0);

    try {
        path.approach_angle = direction.angleTo(vertical);
    } catch (...) {
        path.approach_angle = 0.0;
    }

    path.is_safe = isPathClear(entry_point, target_point);

    if (!path.is_safe) {
        path.safety_notes = "Path intersects with " + std::to_string(obstacles_.size()) + " obstacle(s)";
    } else {
        path.safety_notes = "Path clear - no obstacles detected";
    }

    return path;
}

std::vector<SurgicalPath> PathPlanner::findAlternativePaths(
    const geometry::Point3D& target_point,
    double search_radius,
    int num_candidates
) const {
    std::vector<SurgicalPath> safe_paths;

    const double angle_step = 2.0 * M_PI / num_candidates;

    for (int i = 0; i < num_candidates; ++i) {
        const double angle = i * angle_step;
        const double x_offset = search_radius * std::cos(angle);
        const double y_offset = search_radius * std::sin(angle);

        geometry::Point3D entry(
            target_point.x() + x_offset,
            target_point.y() + y_offset,
            0.0
        );

        SurgicalPath candidate = planStraightPath(entry, target_point);

        if (candidate.is_safe) {
            safe_paths.push_back(candidate);
        }
    }

    std::sort(safe_paths.begin(), safe_paths.end(),
        [](const SurgicalPath& a, const SurgicalPath& b) {
            return a.total_length < b.total_length;
        });

    return safe_paths;
}

} // namespace path
} // namespace surgical_path_planner
