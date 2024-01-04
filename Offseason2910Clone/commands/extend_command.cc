#include "Offseason2910Clone/commands/extend_command.h"

#include "Offseason2910Clone/subsystems/robot_container.h"

ExtendCommand::ExtendCommand(
    RobotContainer& container,
    const transitionLib846::Pref<units::degree_t>& pivot_target,
    const transitionLib846::Pref<units::inch_t>& telescope_target,
    const transitionLib846::Pref<units::degree_t>& wrist_target)
    : pivot_(container.pivot_),
      telescope_(container.telescope_),
      wrist_(container.wrist_),
      pivot_target_(pivot_target),
      telescope_target_(telescope_target),
      wrist_target_(wrist_target) {
  AddRequirements({&pivot_, &telescope_, &wrist_});
  SetName("extend_command");
}

void ExtendCommand::Execute() {
  if (pivot_.Initialized()) {
    pivot_.SetTarget({
        pivot_.subsystem()->ToPosition(pivot_target_),
    });
  }
  if (telescope_.Initialized() && pivot_.Initialized() &&
      ((-pivot_.readings().position + pivot_target_.value() < 32_deg) ||
       pivot_target_.value() < 80_deg)) {
    telescope_.SetTarget({
        telescope_.subsystem()->ToPosition(telescope_target_),
    });
  }
  if (wrist_.Initialized()) {
    wrist_.SetTarget({
        wrist_.subsystem()->ToPosition(wrist_target_),
    });
  }
}

void ExtendCommand::End(bool interrupted) { (void)interrupted; }

bool ExtendCommand::IsFinished() {
  return (units::math::abs(
              (pivot_.readings().position - pivot_target_.value())) < 5_deg) &&
         (units::math::abs(telescope_.readings().extension -
                           telescope_target_.value())) < 2_in;
}