#include "surgical_path_planner/geometry/point3d.hpp"
#include <sstream>
#include <iomanip>

namespace surgical_path_planner {
namespace geometry {

void Point3D::validateCoordinate(double value, const std::string& name) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "Invalid " + name + " coordinate: must be finite. Value: " + std::to_string(value)
        );
    }
}

Point3D::Point3D(double x, double y, double z) : coordinates_{{x, y, z}} {
    validateCoordinate(x, "x");
    validateCoordinate(y, "y");
    validateCoordinate(z, "z");
}

void Point3D::setX(double value) {
    validateCoordinate(value, "x");
    coordinates_[0] = value;
}

void Point3D::setY(double value) {
    validateCoordinate(value, "y");
    coordinates_[1] = value;
}

void Point3D::setZ(double value) {
    validateCoordinate(value, "z");
    coordinates_[2] = value;
}

double Point3D::distanceTo(const Point3D& other) const noexcept {
    const double dx = coordinates_[0] - other.coordinates_[0];
    const double dy = coordinates_[1] - other.coordinates_[1];
    const double dz = coordinates_[2] - other.coordinates_[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string Point3D::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "(" << coordinates_[0] << ", " 
        << coordinates_[1] << ", " 
        << coordinates_[2] << ")";
    return oss.str();
}

} // namespace geometry
} // namespace surgical_path_planner
