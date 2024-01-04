#ifndef Y2023_COMMANDS_INTAKE_COMMAND_H_
#define Y2023_COMMANDS_INTAKE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "y2023/subsystems/intake.h"
#include "y2023/subsystems/robot_container.h"

class IntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeCommand> {
 public:
  IntakeCommand(RobotContainer& container, IntakeState intake_state,
                const frc846::Pref<double>& roller_speed);

  void Execute() override;

  void End(bool interrupted) override;

 private:
  OptionalIntakeSubsystem& intake_;
  OptionalPivotSubsystem& pivot_;
  IntakeState intake_state_;
  const frc846::Pref<double>& roller_speed_;
};
#endif  // Y2023_COMMANDS_INTAKE_COMMAND_H_
