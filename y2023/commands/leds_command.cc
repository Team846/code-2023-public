#include "y2023/commands/leds_command.h"

#include "y2023/subsystems/leds.h"

LEDsCommand::LEDsCommand(RobotContainer& container)
    : gripper_(container.gripper_),
      leds_(container.leds_),
      operator_(container.operator_),
      telescope_(container.telescope_) {
  AddRequirements({&leds_});
  SetName("leds_command");
}

void LEDsCommand::Execute() {
  LEDsTarget target;
  target.has_zeroed = telescope_.subsystem()->GetHasZeroed();
  if (leds_.Initialized()) {
    target.human_player =
        operator_.readings().left_trigger ? LEDsState::kCube : LEDsState::kCone;
  }
  if (gripper_.Initialized()) {
    target.driver = gripper_.readings().has_piece ? LEDsState::kHasPiece
                                                  : LEDsState::kNoPiece;
  }
  leds_.SetTarget({target});
}

void LEDsCommand::End(bool interrupted) {
  (void)interrupted;
  if (leds_.Initialized()) {
    leds_.SetTargetZero();
  }
}

bool LEDsCommand::IsFinished() { return false; }