#ifndef Y2023_OFF_COMMANDS_AUTO_BALANCE_COMMAND_H_
#define Y2023_OFF_COMMANDS_AUTO_BALANCE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023_off/subsystems/drivetrain.h"
#include "y2023_off/subsystems/robot_container.h"

class AutoBalanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
 public:
  AutoBalanceCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;
  bool is_balanced_ = false;
};

#endif  // Y2023_OFF_COMMANDS_AUTO_BALANCE_COMMAND_H_