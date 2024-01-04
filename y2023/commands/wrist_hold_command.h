#ifndef Y2023_COMMANDS_WRIST_HOLD_COMMAND_H_
#define Y2023_COMMANDS_WRIST_HOLD_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/wrist.h"

class WristHoldCommand
    : public frc2::CommandHelper<frc2::CommandBase, WristHoldCommand> {
 public:
  WristHoldCommand(RobotContainer& container);

  void Execute() override;

  void End(bool interrupted) override;

 private:
  OptionalWristSubsystem& wrist_;
};
#endif  // Y2023_COMMANDS_WRIST_HOLD_COMMAND_H_
