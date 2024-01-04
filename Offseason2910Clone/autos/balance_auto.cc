#include "Offseason2910Clone/autos/balance_auto.h"

#include <frc2/command/Commands.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Offseason2910Clone/commands/auto_balance_command.h"
#include "Offseason2910Clone/commands/drive_till_tilt.h"
#include "Offseason2910Clone/commands/extend_command.h"
#include "Offseason2910Clone/commands/follow_trajectory_command.h"
#include "Offseason2910Clone/commands/roller_command.h"
#include "Offseason2910Clone/field.h"
#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "Offseason2910Clone/subsystems/swerve_module.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "transitionLib846/math.h"


BalanceAuto::BalanceAuto(RobotContainer& container, bool should_flip)
    : should_flip_(should_flip) {
  SetName("BalanceAutoCommand" + std::string(should_flip_ ? " red" : " blue"));
  AddCommands(
      frc2::InstantCommand{[&, flip = should_flip_] {
        auto pose_ = field::points::kStartBalance(flip);
        container.drivetrain_.SetPoint(pose_.point);
        container.drivetrain_.SetBearing(pose_.bearing);
      }},

      ExtendCommand{container, container.pivot_.subsystem()->cone_high_,
                    container.telescope_.subsystem()->cone_high_position_,
                    container.wrist_.subsystem()->cone_high_position_},
      frc2::ParallelDeadlineGroup{frc2::WaitCommand{0.5_s},
                                  RollerCommand{container, false, true}},
      ExtendCommand{container, container.pivot_.subsystem()->stow_position_,
                    container.telescope_.subsystem()->retract_position_,
                    container.wrist_.subsystem()->idle_position_},

      DriveTillTilt{container, -5_deg, false, 7_fps},

      DriveTillTilt{container, -3_deg, true, 4_fps},

      /// Going further (till community)
      frc2::ParallelDeadlineGroup{
          frc2::WaitCommand{0.5_s},
          DriveTillTilt{container, 1000_deg, true, 3_fps}},

      DriveTillTilt{container, -11_deg, false, -6_fps},

      frc2::ParallelDeadlineGroup{
          frc2::WaitCommand{2.5_s},
          DriveTillTilt{container, 1000_deg, true, -2_fps}},

      frc2::InstantCommand{[&, flip = should_flip_] {
        transitionLib846::Named("testing_auto").Debug("to balance");
      }},

      AutoBalanceCommand{container});
}