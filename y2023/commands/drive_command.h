#ifndef Y2023_COMMANDS_DRIVE_COMMAND_H_
#define Y2023_COMMANDS_DRIVE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/driver.h"
#include "y2023/subsystems/drivetrain.h"
#include "y2023/subsystems/robot_container.h"

class DriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveCommand> {
 public:
  DriveCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  DrivetrainSubsystem& drivetrain_;
  OptionalLimelightSubsystem& limelight_;
};

#endif  // Y2023_COMMANDS_DRIVE_COMMAND_H_