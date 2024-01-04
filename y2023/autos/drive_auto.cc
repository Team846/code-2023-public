#include "y2023/autos/drive_auto.h"

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
#include "y2023/commands/pickup_conveyor_command.h"
#include "y2023/commands/place_command.h"
#include "y2023/commands/position_telescope_command.h"
#include "y2023/commands/retract_command.h"
#include "y2023/field.h"
#include "y2023/subsystems/drivetrain.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/swerve_module.h"

DriveAuto::DriveAuto(
    RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("ScoringTwoPieceAutoCommand" +
          std::string(should_flip_ ? " red" : " blue"));
  AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kStartScoringTable(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
      }},


      // Align to cone node
      FollowTrajectoryCommand{
          container,
          {
              {field::points::kTestingPoint(should_flip_), 0_fps},
          }
    });
}