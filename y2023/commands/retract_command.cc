#include "y2023/commands/retract_command.h"

#include "y2023/subsystems/robot_container.h"

RetractCommand::RetractCommand(
    RobotContainer& container,
    const frc846::Pref<units::degree_t>& pivot_target,
    const frc846::Pref<units::turn_t>& telescope_target)
    : telescope_(container.telescope_),
      pivot_(container.pivot_),
      pivot_target_(pivot_target),
      telescope_target_(telescope_target) {
  AddRequirements({&telescope_, &pivot_});
  SetName("retract_command");
}

void RetractCommand::Execute() {
  if (telescope_.Initialized() && pivot_.Initialized()) {
    if (units::math::abs(telescope_.readings().position -
                         telescope_target_.value()) <
        telescope_.subsystem()->max_retract_position_.value()) {
      // Only retract once clearing a certain point (bumpers + shelf)
      telescope_.SetTarget({
          telescope_target_.value(),
      });
      pivot_.SetTarget({
          pivot_target_.value(),
      });
    } else {
      telescope_.SetTarget({
          telescope_target_.value(),
      });
      pivot_.SetTargetZero();
    }
  }
}

void RetractCommand::End(bool interrupted) {
  (void)interrupted;
  if (telescope_.Initialized() && pivot_.Initialized()) {
    telescope_.SetTarget(telescope_.subsystem()->HoldPosition());
    pivot_.SetTarget(pivot_.subsystem()->HoldPosition());
  }
}

bool RetractCommand::IsFinished() { return false; }