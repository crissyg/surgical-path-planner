#include "surgical_path_planner/geometry/vector3d.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace surgical_path_planner {
namespace geometry {

void Vector3D::validateComponent(double value, const std::string& name) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "Invalid " + name + " component: must be finite. Value: " + std::to_string(value)
        );
    }
}

Vector3D::Vector3D(double x, double y, double z) : components_{{x, y, z}} {
    validateComponent(x, "x");
    validateComponent(y, "y");
    validateComponent(z, "z");
}

Vector3D Vector3D::fromPoints(const Point3D& from, const Point3D& to) noexcept {
    return Vector3D(to.x() - from.x(), to.y() - from.y(), to.z() - from.z());
}

double Vector3D::magnitude() const noexcept {
    return std::sqrt(components_[0] * components_[0] +
                     components_[1] * components_[1] +
                     components_[2] * components_[2]);
}

Vector3D Vector3D::normalized() const {
    const double mag = magnitude();
    if (mag == 0.0) {
        throw std::runtime_error("Cannot normalize zero vector");
    }
    return Vector3D(components_[0] / mag, components_[1] / mag, components_[2] / mag);
}

double Vector3D::dot(const Vector3D& other) const noexcept {
    return components_[0] * other.components_[0] +
           components_[1] * other.components_[1] +
           components_[2] * other.components_[2];
}

Vector3D Vector3D::cross(const Vector3D& other) const noexcept {
    return Vector3D(
        components_[1] * other.components_[2] - components_[2] * other.components_[1],
        components_[2] * other.components_[0] - components_[0] * other.components_[2],
        components_[0] * other.components_[1] - components_[1] * other.components_[0]
    );
}

double Vector3D::angleTo(const Vector3D& other) const {
    const double mag1 = magnitude();
    const double mag2 = other.magnitude();

    if (mag1 == 0.0 || mag2 == 0.0) {
        throw std::runtime_error("Cannot compute angle with zero vector");
    }

    const double dot_product = dot(other);
    const double cos_angle = dot_product / (mag1 * mag2);
    const double clamped = std::max(-1.0, std::min(1.0, cos_angle));

    return std::acos(clamped);
}

Vector3D Vector3D::operator+(const Vector3D& other) const noexcept {
    return Vector3D(components_[0] + other.components_[0],
                    components_[1] + other.components_[1],
                    components_[2] + other.components_[2]);
}

Vector3D Vector3D::operator-(const Vector3D& other) const noexcept {
    return Vector3D(components_[0] - other.components_[0],
                    components_[1] - other.components_[1],
                    components_[2] - other.components_[2]);
}

Vector3D Vector3D::operator*(double scalar) const {
    validateComponent(scalar, "scalar");
    return Vector3D(components_[0] * scalar,
                    components_[1] * scalar,
                    components_[2] * scalar);
}

std::string Vector3D::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "[" << components_[0] << ", " 
        << components_[1] << ", " 
        << components_[2] << "]";
    return oss.str();
}

} // namespace geometry
} // namespace surgical_path_planner
