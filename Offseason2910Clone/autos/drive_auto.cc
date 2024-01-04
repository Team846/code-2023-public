#include "Offseason2910Clone/autos/drive_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Offseason2910Clone/commands/follow_trajectory_command.h"
#include "Offseason2910Clone/field.h"
#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "Offseason2910Clone/subsystems/swerve_module.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "transitionLib846/math.h"


DriveAuto::DriveAuto(RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("DriveAutoCommand" + std::string(should_flip_ ? " red" : " blue"));
  AddCommands(frc2::InstantCommand{[&, flip = should_flip_] {
                auto pose_ = field::points::kStartScoringTable(flip);
                container.drivetrain_.SetPoint(pose_.point);
                container.drivetrain_.SetBearing(pose_.bearing);
              }},

              // Align to cone node
              FollowTrajectoryCommand{
                  container,
                  {
                      {field::points::kTestingPoint(should_flip_), 0_fps},
                  }});
}