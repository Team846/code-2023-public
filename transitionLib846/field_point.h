#ifndef transitionLib846_FIELD_POINT_H_
#define transitionLib846_FIELD_POINT_H_

#include "transitionLib846/pref.h"
#include "transitionLib846/math.h"
#include "transitionLib846/named.h"
#include <units/length.h>
#include <units/angle.h>

namespace transitionLib846::field {

class FieldPoint {
  public:

  transitionLib846::Position to_position(FieldPoint point) {
    return Position{{point.x.value(), point.y.value()}, point.bearing.value()};
  }

  FieldPoint flip(
      FieldPoint point, bool should_flip) {
    if (should_flip) {
      return FieldPoint{point.name_, -point.x.value(), point.y.value(), flip(point.bearing.value(), should_flip)};
    } else {
      return FieldPoint{point.name_, point.x.value(), point.y.value(), flip(point.bearing.value(), should_flip)};
    }
  }


  units::degree_t flip(units::degree_t bearing, bool should_flip) {
    if (should_flip) {
      return -bearing;
    } else {
      return bearing;
    }
  }

  Position flip(bool should_flip) {
    return to_position(flip(*this, should_flip));
  }

  FieldPoint(std::string name, units::inch_t point_x, units::inch_t point_y, units::degree_t bearing_) : 
    x{point_names_, name + "_x", point_x},
    y{point_names_, name + "_y", point_y},
    bearing{point_names_, name + "_deg", bearing_},
    name_(name) {
      frc::Preferences::SetDouble(x.key, point_x.to<double>());
      frc::Preferences::SetDouble(y.key, point_y.to<double>());
      frc::Preferences::SetDouble(bearing.key, bearing_.to<double>());
  }

  FieldPoint(std::string name, transitionLib846::Vector2D<units::inch_t> point, units::degree_t bearing_) : 
    x{point_names_, name + "_x", point.x},
    y{point_names_, name + "_y", point.y},
    bearing{point_names_, name + "_deg", bearing_},
    name_(name) {
      frc::Preferences::SetDouble(x.key, point.x.to<double>());
      frc::Preferences::SetDouble(y.key, point.y.to<double>());
      frc::Preferences::SetDouble(bearing.key, bearing_.to<double>());
  }

    Named& point_names_ = *(new Named("field_points"));

    Pref<units::inch_t> x;
    Pref<units::inch_t> y;
    Pref<units::degree_t> bearing;
    std::string name_;

  private:

};

}

#endif  // transitionLib846_FIELD_POINT_H_