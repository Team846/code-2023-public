#ifndef Offseason2910Clone_COMMANDS_ROLLER_COMMAND_H_
#define Offseason2910Clone_COMMANDS_ROLLER_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "Offseason2910Clone/subsystems/robot_container.h"
#include "Offseason2910Clone/subsystems/roller.h"

class RollerCommand
    : public frc2::CommandHelper<frc2::CommandBase, RollerCommand> {
 public:
  RollerCommand(RobotContainer& container, const bool grab, const bool cone);

  void Initialize() override;

  void End(bool interrupted) override;

 private:
  OptionalRollerSubsystem& roller_;
  const bool grab_;
  const bool cone_;
};
#endif  // Offseason2910Clone_COMMANDS_ROLLER_COMMAND_H_
