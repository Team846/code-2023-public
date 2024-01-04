#include "y2023/commands/wrist_command.h"

WristCommand::WristCommand(RobotContainer& container,
                               const frc846::Pref<double>& speed)
    : wrist_(container.wrist_), speed_(speed) {
  AddRequirements({&wrist_});
  SetName("wrist_command");
}

void WristCommand::Initialize() {
  if (wrist_.Initialized()) {
    wrist_.SetTarget({speed_.value()});
  }
}

void WristCommand::End(bool interrupted) {
  (void)interrupted;
  if (wrist_.Initialized()) {
    wrist_.SetTargetZero();
  }
}