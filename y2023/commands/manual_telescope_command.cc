#include "y2023/commands/manual_telescope_command.h"

#include "y2023/subsystems/robot_container.h"

ManualTelescopeCommand::ManualTelescopeCommand(
    RobotContainer& container, const frc846::Pref<double>& target)
    : telescope_(container.telescope_), target_(target) {
  AddRequirements({&telescope_});
  SetName("manual_telescope_command");
}

void ManualTelescopeCommand::Execute() {
  if (telescope_.Initialized()) {
    telescope_.SetTarget({
        target_.value(),
    });
  }
}

void ManualTelescopeCommand::End(bool interrupted) {
  (void)interrupted;
  if (telescope_.Initialized()) {
    telescope_.SetTarget({telescope_.readings().position});
  }
}

bool ManualTelescopeCommand::IsFinished() { return false; }