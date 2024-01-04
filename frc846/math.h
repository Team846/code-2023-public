#ifndef FRC846_MATH_H_
#define FRC846_MATH_H_

#include <units/angle.h>
#include <units/constants.h>
#include <units/length.h>
#include <units/math.h>

#include <cmath>

namespace frc846 {

// Find the circumference of a circle given radius.
constexpr units::inch_t Circumference(units::meter_t radius) {
  return 2 * units::constants::pi * radius;
}

double HorizontalDeadband(double input, double x_intercept, double max,
                          double exponent = 1, double sensitivity = 1);

double VerticalDeadband(double input, double y_intercept, double max,
                        double exponent = 1, double sensitivity = 1);

// Returns the smallest difference between an element of C(θ₁) and an element of
// C(θ₂).
units::degree_t CoterminalDifference(units::degree_t angle,
                                     units::degree_t other_angle);

// Returns the smallest sum between two angles.
units::degree_t CoterminalSum(units::degree_t angle,
                              units::degree_t other_angle);

// A 2D vector.
//
// TODO only calculate magnitude and bearing once
template <class T>
struct Vector2D {
  static_assert(units::traits::is_unit_t<T>(), "must be a unit");

  T x;
  T y;

  // Magnitude of the vector.
  T Magnitude() const { return units::math::sqrt(x * x + y * y); }

  // Bearing of the vector.
  //
  // x and y are intentionally swapped in atan2.
  units::radian_t Bearing() const { return units::math::atan2(x, y); }

  // Returns a new vector rotate CLOCKWISE by theta.
  Vector2D<T> Rotate(units::degree_t theta) const {
    return {
        x * units::math::cos(theta) + y * units::math::sin(theta),
        x * -units::math::sin(theta) + y * units::math::cos(theta),
    };
  }

  Vector2D<T> Extrapolate(Vector2D<T> from, units::inch_t by) const {
    auto bearing = units::math::atan2(x - from.x, y - from.y);
    return {x + by * units::math::sin(bearing),
            y + by * units::math::cos(bearing)};
  }

  Vector2D<T> operator+(const Vector2D& other) const {
    return {x + other.x, y + other.y};
  }

  Vector2D<T> operator-(const Vector2D& other) const {
    return {x - other.x, y - other.y};
  }

  Vector2D<T> operator*(double scalar) const {
    return {x * scalar, y * scalar};
  }

  Vector2D<T> operator/(double scalar) const {
    return {x / scalar, y / scalar};
  }

  bool operator==(const Vector2D& other) const {
    return x == other.x && y == other.y;
  }
};

struct Position {
  Vector2D<units::foot_t> point;
  units::degree_t bearing;
};

}  // namespace frc846

#endif  // FRC846_MATH_H_