#ifndef Offseason2910Clone_COMMANDS_LEDS_COMMAND_H_
#define Offseason2910Clone_COMMANDS_LEDS_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/leds.h"
#include "Offseason2910Clone/subsystems/operator.h"
#include "Offseason2910Clone/subsystems/robot_container.h"
#include "Offseason2910Clone/subsystems/roller.h"
#include "Offseason2910Clone/subsystems/telescope.h"


class LEDsCommand : public frc2::CommandHelper<frc2::CommandBase, LEDsCommand> {
 public:
  LEDsCommand(RobotContainer& container);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalLEDsSubsystem& leds_;
  OptionalRollerSubsystem& roller_;
  OptionalTelescopeSubsystem& telescope_;
  OperatorSubsystem& operator_;
};
#endif  // Offseason2910Clone_COMMANDS_LEDS_COMMAND_H_
