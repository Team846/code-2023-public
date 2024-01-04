#ifndef Y2023_COMMANDS_MANUAL_PIVOT_COMMAND_H_
#define Y2023_COMMANDS_MANUAL_PIVOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/subsystem.h"
#include "y2023/subsystems/robot_container.h"
#include "y2023/subsystems/pivot.h"

class ManualPivotCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualPivotCommand> {
 public:
  ManualPivotCommand(RobotContainer& container,
                         const frc846::Pref<double>& target);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalPivotSubsystem& pivot_;

  const frc846::Pref<double>& target_;
};

#endif  // Y2023_COMMANDS_MANUAL_PIVOT_COMMAND_H_