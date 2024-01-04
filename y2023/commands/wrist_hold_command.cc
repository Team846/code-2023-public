#include "y2023/commands/wrist_hold_command.h"

WristHoldCommand::WristHoldCommand(RobotContainer& container)
    : wrist_(container.wrist_) {
  AddRequirements({&wrist_});
  SetName("wrist_hold_command");
}

void WristHoldCommand::Execute() {
  if (wrist_.Initialized()) {
    if (wrist_.subsystem()->prev_state_ == kForward) {
      wrist_.SetTarget({wrist_.subsystem()->hold_speed_.value()});
    } else {
      wrist_.SetTarget({-1 * wrist_.subsystem()->hold_speed_.value()});
    }
  }
}

void WristHoldCommand::End(bool interrupted) {
  (void)interrupted;
  if (wrist_.Initialized()) {
    wrist_.SetTargetZero();
  }
}