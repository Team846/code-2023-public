#ifndef Offseason2910Clone_COMMANDS_DRIVE_COMMAND_H_
#define Offseason2910Clone_COMMANDS_DRIVE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/driver.h"
#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "Offseason2910Clone/subsystems/roller.h"

class DriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveCommand> {
 public:
  DriveCommand(RobotContainer& container);

  double JoystickOutput(double ji);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  DrivetrainSubsystem& drivetrain_;
  OptionalLimelightSubsystem& limelight_;
  OptionalRollerSubsystem& roller_;

  bool loop_b_unheld = true;
  bool toggle_align_to_shelf = false;
};

#endif  // Offseason2910Clone_COMMANDS_DRIVE_COMMAND_H_