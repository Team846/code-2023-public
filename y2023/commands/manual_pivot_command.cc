#include "y2023/commands/manual_pivot_command.h"

#include "y2023/subsystems/robot_container.h"

ManualPivotCommand::ManualPivotCommand(RobotContainer& container,
                                       const frc846::Pref<double>& target)
    : pivot_(container.pivot_), target_(target) {
  AddRequirements({&pivot_});
  SetName("manual_pivot_command");
}

void ManualPivotCommand::Execute() {
  if (pivot_.Initialized()) {
    pivot_.SetTarget({
        target_.value(),
    });
  }
}

void ManualPivotCommand::End(bool interrupted) {
  (void)interrupted;
  if (pivot_.Initialized()) {
    pivot_.SetTarget({pivot_.readings().position});
  }
}

bool ManualPivotCommand::IsFinished() { return false; }