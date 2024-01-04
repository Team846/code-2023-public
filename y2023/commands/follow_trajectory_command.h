#ifndef Y2023_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_
#define Y2023_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/math.h"
#include "frc846/trajectory_generator.h"
#include "y2023/subsystems/drivetrain.h"
#include "y2023/subsystems/robot_container.h"

class FollowTrajectoryCommand
    : public frc2::CommandHelper<frc2::CommandBase, FollowTrajectoryCommand>,
      public frc846::Named {
 public:
  FollowTrajectoryCommand(RobotContainer& container,
                          std::vector<frc846::InputWaypoint> input_points);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;

  std::vector<frc846::InputWaypoint> input_points_;
  frc846::Trajectory trajectory_;

  unsigned int target_idx_ = 1;
  bool is_done_ = false;
  frc846::Vector2D<units::foot_t> current_extrapolated_point_;

  units::second_t start_time_;

  static bool HasCrossedWaypoint(frc846::Waypoint current_waypoint,
                                 frc846::Waypoint prev_waypoint,
                                 frc846::Vector2D<units::foot_t> pos,
                                 frc846::Vector2D<units::foot_t> test_target);
};

#endif  // Y2023_COMMANDS_FOLLOW_TRAJECTORY_COMMAND_H_