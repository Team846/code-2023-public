#include "y2023/commands/place_command.h"

#include "y2023/subsystems/pivot.h"
#include "y2023/subsystems/robot_container.h"

PlaceCommand::PlaceCommand(RobotContainer& container)
    : pivot_(container.pivot_) {
  AddRequirements({&pivot_});
  SetName("place_command");
}

void PlaceCommand::Initialize() {
  if (pivot_.Initialized()) {
    pivot_.SetTarget({pivot_.readings().position - pivot_.subsystem()->place_position_.value()});
  }
}

void PlaceCommand::End(bool interrupted) {
  (void)interrupted;
  if (pivot_.Initialized()) {
    pivot_.SetTarget(pivot_.subsystem()->HoldPosition());
  }
}