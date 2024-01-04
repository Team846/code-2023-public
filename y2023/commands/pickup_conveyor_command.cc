#include "y2023/commands/pickup_conveyor_command.h"

#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/WaitCommand.h"
#include "frc2/command/WaitUntilCommand.h"
#include "frc846/pref.h"
#include "y2023/commands/position_pivot_command.h"
#include "y2023/commands/position_telescope_command.h"
#include "y2023/subsystems/robot_container.h"

PickupConveyorCommand::PickupConveyorCommand(
    RobotContainer& container, const frc846::Pref<bool>& should_auto_pickup)
    : gripper_(container.gripper_), should_auto_pickup_(should_auto_pickup) {
  AddRequirements({
      &gripper_,
  });
  SetName("pickup_conveyor_command");
}

void PickupConveyorCommand::Execute() {
  if (gripper_.Initialized()) {
    // Keep rollers running at constant speed

    if (should_auto_pickup_.value() &&
        !gripper_.subsystem()->readings().has_piece) {
      gripper_.SetTarget({gripper_.subsystem()->grab_speed_.value()});
    } else {
      gripper_.SetTargetZero();
    }
  }
}

void PickupConveyorCommand::End(bool interrupted) { (void)interrupted; }

bool PickupConveyorCommand::IsFinished() { return false; }