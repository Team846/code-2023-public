#include "y2023/autos/mid_one_piece_pickup_dock_auto_command.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/math.h"
#include "y2023/commands/auto_balance_command.h"
#include "y2023/commands/auto_extend_command.h"
#include "y2023/commands/conveyor_command.h"
#include "y2023/commands/extend_command.h"
#include "y2023/commands/follow_trajectory_command.h"
#include "y2023/commands/gripper_command.h"
#include "y2023/commands/intake_command.h"
#include "y2023/commands/load_conveyor_command.h"
#include "y2023/commands/place_command.h"
#include "y2023/commands/position_pivot_command.h"
#include "y2023/commands/position_telescope_command.h"
#include "y2023/commands/retract_command.h"
#include "y2023/field.h"
#include "y2023/subsystems/robot_container.h"

MidOnePiecePickupDockAutoCommand::MidOnePiecePickupDockAutoCommand(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("MidOnePiecePickupDockAutoCommand" +
          std::string(should_flip_ ? " red" : " blue"));
  AddCommands(
      // SetPose
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kStartMid(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
      }},

      // Move to raise position & retract arm in case it's fallen
      frc2::ParallelDeadlineGroup{
          FollowTrajectoryCommand{
              container, {{field::points::kMidRaise(should_flip_), 0_fps}}},
          PositionTelescopeCommand{
              container,
              container.telescope_.subsystem()->cube_conveyor_position_}},
      frc2::ParallelRaceGroup{
          frc2::WaitCommand{2_s},
          AutoExtendCommand{
              container, container.pivot_.subsystem()->cone_high_position_,
              container.telescope_.subsystem()->cone_high_position_,
              container.pivot_.subsystem()->cone_min_mid_}},
      FollowTrajectoryCommand{
          container, {{field::points::kStartMid(should_flip_), 0_fps}}},
      frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.75_s},
                                  PlaceCommand{container}},
      frc2::ParallelDeadlineGroup{
          frc2::WaitCommand{0.1_s},
          GripperCommand{container,
                         container.gripper_.subsystem()->place_speed_}},
      frc2::ParallelDeadlineGroup{
          FollowTrajectoryCommand{
              container,
              {{field::points::kBalanceBackward(should_flip_), 0_fps},
               {field::points::kIntake2(should_flip_), 0_fps}}},
          RetractCommand{
              container, container.pivot_.subsystem()->cube_conveyor_position_,
              container.telescope_.subsystem()->cube_conveyor_position_},
          GripperCommand{container,
                         container.gripper_.subsystem()->place_speed_}},
      frc2::ParallelDeadlineGroup{

          FollowTrajectoryCommand{
              container,
              {{field::points::kBalanceBackward(should_flip_), 0_fps}},
          },
          RetractCommand{
              container, container.pivot_.subsystem()->cube_conveyor_position_,
              container.telescope_.subsystem()->cube_conveyor_position_},
      },
      AutoBalanceCommand{container});
}