#ifndef transitionLib846_TRAJECTORY_GENERATOR_H_
#define transitionLib846_TRAJECTORY_GENERATOR_H_

#include <optional>
#include <vector>

#include "transitionLib846/math.h"

namespace transitionLib846 {

struct InputWaypoint {
  transitionLib846::Position pos;
  std::optional<units::feet_per_second_t> v_max;
};

struct Waypoint {
  transitionLib846::Position pos;
  units::feet_per_second_t v;
};

using Trajectory = std::vector<Waypoint>;

std::vector<Position> InterpolatePoints(Vector2D<units::foot_t> start,
                                        Vector2D<units::foot_t> end,
                                        units::degree_t start_bearing,
                                        units::degree_t end_bearing,
                                        units::foot_t cut);

Trajectory GenerateTrajectory(
    std::vector<InputWaypoint> input_points,
    units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut = 6_in);

}  // namespace transitionLib846

#endif  // transitionLib846_TRAJECTORY_GENERATOR_H_