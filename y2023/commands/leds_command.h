#ifndef Y2023_COMMANDS_LEDS_COMMAND_H_
#define Y2023_COMMANDS_LEDS_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/gripper.h"
#include "y2023/subsystems/leds.h"
#include "y2023/subsystems/operator.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/telescope.h"

class LEDsCommand : public frc2::CommandHelper<frc2::CommandBase, LEDsCommand> {
 public:
  LEDsCommand(RobotContainer& container);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalGripperSubsystem& gripper_;
  OptionalLEDsSubsystem& leds_;
  OperatorSubsystem& operator_;
  OptionalTelescopeSubsystem& telescope_;
};
#endif  // Y2023_COMMANDS_LEDS_COMMAND_H_
