#include "y2023/commands/conveyor_command.h"

#include "y2023/subsystems/conveyor.h"

ConveyorCommand::ConveyorCommand(RobotContainer& container,
                                 const frc846::Pref<double>& belt_speed,
                                 const frc846::Pref<double>& roller_speed)
    : conveyor_(container.conveyor_),
      belt_speed_(belt_speed),
      roller_speed_(roller_speed) {
  AddRequirements({&conveyor_});
  SetName("conveyor_command");
}

void ConveyorCommand::Execute() {
  if (conveyor_.Initialized()) {
    ConveyorTarget target;

    auto default_roller_speed = roller_speed_.value();
    target.left_roller_speed = default_roller_speed;
    target.right_roller_speed = default_roller_speed;

    target.belt_speed = belt_speed_.value();

    conveyor_.SetTarget(target);
  }
}

void ConveyorCommand::End(bool interrupted) {
  (void)interrupted;
  if (conveyor_.Initialized()) {
    conveyor_.SetTargetZero();
  }
}

bool ConveyorCommand::IsFinished() { return false; }