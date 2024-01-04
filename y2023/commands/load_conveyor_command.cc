#include "y2023/commands/load_conveyor_command.h"

#include "y2023/subsystems/conveyor.h"

LoadConveyorCommand::LoadConveyorCommand(RobotContainer& container,
                                         const frc846::Pref<bool>& should_run)
    : conveyor_(container.conveyor_),
      gripper_(container.gripper_),
      should_run_(should_run) {
  AddRequirements({&conveyor_});
  SetName("load_conveyor_command");
}

void LoadConveyorCommand::Execute() {
  if (conveyor_.Initialized()) {
    ConveyorTarget target;

    // Keep rollers running at constant speed
    auto default_roller_speed =
        conveyor_.subsystem()->default_roller_speed_.value();
    target.left_roller_speed = default_roller_speed;
    target.right_roller_speed = default_roller_speed;

    bool should_run = should_run_.value();

    if (should_run && !gripper_.subsystem()->readings().has_piece) {
      target.belt_speed = conveyor_.subsystem()->belt_speed_.value();
      target.left_roller_speed = default_roller_speed;
      target.right_roller_speed = default_roller_speed;
    } else {
      target.left_roller_speed = 0;
      target.right_roller_speed = 0;
      target.belt_speed = 0;
    }

    conveyor_.SetTarget(target);
  }
}

void LoadConveyorCommand::End(bool interrupted) {
  (void)interrupted;
  if (conveyor_.Initialized()) {
    conveyor_.SetTargetZero();
  }
}

bool LoadConveyorCommand::IsFinished() { return false; }