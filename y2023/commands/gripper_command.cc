#include "y2023/commands/gripper_command.h"

GripperCommand::GripperCommand(RobotContainer& container,
                               const frc846::Pref<double>& gripper_speed)
    : gripper_(container.gripper_), gripper_speed_(gripper_speed) {
  AddRequirements({&gripper_});
  SetName("gripper_command");
}

void GripperCommand::Initialize() {
  if (gripper_.Initialized()) {
    gripper_.SetTarget({gripper_speed_.value()});
  }
}

void GripperCommand::End(bool interrupted) {
  (void)interrupted;
  if (gripper_.Initialized()) {
    gripper_.SetTargetZero();
  }
}