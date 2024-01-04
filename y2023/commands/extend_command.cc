#include "y2023/commands/extend_command.h"

#include "frc846/pref.h"
#include "units/math.h"
#include "y2023/subsystems/robot_container.h"

ExtendCommand::ExtendCommand(
    RobotContainer& container,
    const frc846::Pref<units::degree_t>& pivot_target,
    const frc846::Pref<units::turn_t>& telescope_target,
    const frc846::Pref<units::degree_t>& min_pivot)
    : telescope_(container.telescope_),
      pivot_(container.pivot_),
      pivot_target_(pivot_target),
      telescope_target_(telescope_target),
      min_pivot_(min_pivot) {
  AddRequirements({&telescope_, &pivot_});
  SetName("extend_command");
}

void ExtendCommand::Execute() {
  if (telescope_.Initialized() && pivot_.Initialized()) {
    if (pivot_.readings().position > min_pivot_.value()) {
      telescope_.SetTarget({
          telescope_target_.value(),
      });
      pivot_.SetTarget({
          pivot_target_.value(),
      });
    } else {
      telescope_.SetTargetZero();
      pivot_.SetTarget({
          pivot_target_.value(),
      });
    }
  }
}

void ExtendCommand::End(bool interrupted) {
  (void)interrupted;
  if (telescope_.Initialized() && pivot_.Initialized()) {
    telescope_.SetTarget(telescope_.subsystem()->HoldPosition());
    pivot_.SetTarget(pivot_.subsystem()->HoldPosition());
  }
}

bool ExtendCommand::IsFinished() { return false; }