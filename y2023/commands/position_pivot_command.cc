#include "y2023/commands/position_pivot_command.h"

#include "y2023/subsystems/pivot.h"
#include "y2023/subsystems/robot_container.h"

PositionPivotCommand::PositionPivotCommand(
    RobotContainer& container, const frc846::Pref<units::degree_t>& target)
    : pivot_(container.pivot_), target_(target) {
  AddRequirements({&pivot_});
  SetName("position_pivot_command");
}

void PositionPivotCommand::Execute() {
  if (pivot_.Initialized()) {
    pivot_.SetTarget({
        target_.value(),
    });
  }
}

void PositionPivotCommand::End(bool interrupted) {
  (void)interrupted;
  if (pivot_.Initialized()) {
    pivot_.SetTarget(pivot_.subsystem()->HoldPosition());
  }
}

bool PositionPivotCommand::IsFinished() { return false; }