#include "Offseason2910Clone/subsystems/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : transitionLib846::Subsystem<LEDsReadings, LEDsTarget>("leds") {
  leds_.SetLength(kLength);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

LEDsTarget LEDsSubsystem::ZeroTarget() const {
  LEDsTarget target;
  target.state = kCone;
  target.has_zeroed = true;
  return target;
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::GetNewReadings() { return {}; }

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  if (!target.has_zeroed || target.state == kNotZeroed) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(250, 0, 0);
    }
  } else if (target.state == LEDsState::kCone) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(246, 190, 0);
    }
  } else if (target.state == LEDsState::kCube) {
    for (int i = 0; i < kLength; i++) {
      leds_buffer_[i].SetRGB(75, 0, 130);
    }
  }

  leds_.SetData(leds_buffer_);
}
