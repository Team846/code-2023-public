#include "Offseason2910Clone/commands/leds_command.h"

#include "Offseason2910Clone/subsystems/leds.h"

LEDsCommand::LEDsCommand(RobotContainer& container)
    : leds_(container.leds_),
      roller_(container.roller_),
      telescope_(container.telescope_),
      operator_(container.operator_) {
  AddRequirements({&leds_});
  SetName("leds_command");
}

void LEDsCommand::Execute() {
  LEDsTarget target;
  target.has_zeroed = false;
  if (leds_.Initialized()) {
    if (telescope_.Initialized() && telescope_.subsystem()->has_zeroed) {
      if (roller_.Initialized() && roller_.subsystem()->GetCubeMode()) {
        leds_.SetTarget({LEDsState::kCube, true});
      } else {
        leds_.SetTarget({LEDsState::kCone, true});
      }
    } else {
      leds_.SetTarget({LEDsState::kNotZeroed, false});
    }
  }
}

void LEDsCommand::End(bool interrupted) {
  (void)interrupted;
  if (leds_.Initialized()) {
    leds_.SetTargetZero();
  }
}

bool LEDsCommand::IsFinished() { return false; }