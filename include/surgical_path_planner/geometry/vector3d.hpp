#pragma once

#include "surgical_path_planner/geometry/point3d.hpp"
#include <array>
#include <string>

namespace surgical_path_planner {
namespace geometry {

class Vector3D {
public:
    constexpr Vector3D() noexcept : components_{{0.0, 0.0, 0.0}} {}
    Vector3D(double x, double y, double z);

    [[nodiscard]] static Vector3D fromPoints(const Point3D& from, const Point3D& to) noexcept;

    [[nodiscard]] constexpr double x() const noexcept { return components_[0]; }
    [[nodiscard]] constexpr double y() const noexcept { return components_[1]; }
    [[nodiscard]] constexpr double z() const noexcept { return components_[2]; }

    [[nodiscard]] double magnitude() const noexcept;
    [[nodiscard]] Vector3D normalized() const;
    [[nodiscard]] double dot(const Vector3D& other) const noexcept;
    [[nodiscard]] Vector3D cross(const Vector3D& other) const noexcept;
    [[nodiscard]] double angleTo(const Vector3D& other) const;

    [[nodiscard]] Vector3D operator+(const Vector3D& other) const noexcept;
    [[nodiscard]] Vector3D operator-(const Vector3D& other) const noexcept;
    [[nodiscard]] Vector3D operator*(double scalar) const;
    [[nodiscard]] std::string toString() const;

    [[nodiscard]] constexpr bool operator==(const Vector3D& other) const noexcept {
        return components_[0] == other.components_[0] &&
               components_[1] == other.components_[1] &&
               components_[2] == other.components_[2];
    }

    [[nodiscard]] constexpr bool operator!=(const Vector3D& other) const noexcept {
        return !(*this == other);
    }

private:
    std::array<double, 3> components_;
    static void validateComponent(double value, const std::string& name);
};

} // namespace geometry
} // namespace surgical_path_planner
