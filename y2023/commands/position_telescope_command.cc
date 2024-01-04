#include "y2023/commands/position_telescope_command.h"

#include "y2023/subsystems/robot_container.h"

PositionTelescopeCommand::PositionTelescopeCommand(
    RobotContainer& container, const frc846::Pref<units::turn_t>& target)
    : telescope_(container.telescope_), target_(target) {
  AddRequirements({&telescope_});
  SetName("position_telescope_command");
}

void PositionTelescopeCommand::Execute() {
  if (telescope_.Initialized()) {
    telescope_.SetTarget({
        target_.value(),
    });
  }
}

void PositionTelescopeCommand::End(bool interrupted) {
  (void)interrupted;
  if (telescope_.Initialized()) {
    telescope_.SetTarget(telescope_.subsystem()->HoldPosition());
  }
}

bool PositionTelescopeCommand::IsFinished() {
  return (units::math::abs(target_.value() - telescope_.readings().position) <
          telescope_.subsystem()->tolerance_.value());
}