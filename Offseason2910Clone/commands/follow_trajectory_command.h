#ifndef Offseason2910Clone_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_
#define Offseason2910Clone_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "transitionLib846/math.h"
#include "transitionLib846/trajectory_generator.h"


class FollowTrajectoryCommand
    : public frc2::CommandHelper<frc2::CommandBase, FollowTrajectoryCommand>,
      public transitionLib846::Named {
 public:
  FollowTrajectoryCommand(
      RobotContainer& container,
      std::vector<transitionLib846::InputWaypoint> input_points);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;

  std::vector<transitionLib846::InputWaypoint> input_points_;
  transitionLib846::Trajectory trajectory_;

  unsigned int target_idx_ = 1;
  bool is_done_ = false;
  transitionLib846::Vector2D<units::foot_t> current_extrapolated_point_;

  units::second_t start_time_;

  static bool HasCrossedWaypoint(
      transitionLib846::Waypoint current_waypoint,
      transitionLib846::Waypoint prev_waypoint,
      transitionLib846::Vector2D<units::foot_t> pos,
      transitionLib846::Vector2D<units::foot_t> test_target);
};

#endif  // Offseason2910Clone_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_