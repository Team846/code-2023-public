#ifndef Offseason2910Clone_COMMANDS_AUTO_BALANCE_COMMAND_H_
#define Offseason2910Clone_COMMANDS_AUTO_BALANCE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/drivetrain.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "transitionLib846/logger.h"

class AutoBalanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
 public:
  AutoBalanceCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  transitionLib846::Logger* logger =
      new transitionLib846::Logger("auto_balance_command", false, true);

  DrivetrainSubsystem& drivetrain_;
  bool is_balanced_ = false;
  int loops_since = 0;

  units::second_t start_time_ = 0_s;
};

#endif  // Y2023_COMMANDS_AUTO_BALANCE_COMMAND_H_