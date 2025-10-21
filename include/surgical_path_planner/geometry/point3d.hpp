#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <array>

namespace surgical_path_planner {
namespace geometry {

class Point3D {
public:
    constexpr Point3D() noexcept : coordinates_{{0.0, 0.0, 0.0}} {}
    Point3D(double x, double y, double z);

    [[nodiscard]] constexpr double x() const noexcept { return coordinates_[0]; }
    [[nodiscard]] constexpr double y() const noexcept { return coordinates_[1]; }
    [[nodiscard]] constexpr double z() const noexcept { return coordinates_[2]; }

    void setX(double value);
    void setY(double value);
    void setZ(double value);

    [[nodiscard]] double distanceTo(const Point3D& other) const noexcept;
    [[nodiscard]] std::string toString() const;

    [[nodiscard]] constexpr bool operator==(const Point3D& other) const noexcept {
        return coordinates_[0] == other.coordinates_[0] &&
               coordinates_[1] == other.coordinates_[1] &&
               coordinates_[2] == other.coordinates_[2];
    }

    [[nodiscard]] constexpr bool operator!=(const Point3D& other) const noexcept {
        return !(*this == other);
    }

private:
    std::array<double, 3> coordinates_;
    static void validateCoordinate(double value, const std::string& name);
};

} // namespace geometry
} // namespace surgical_path_planner
