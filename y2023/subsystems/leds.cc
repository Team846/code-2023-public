#include "y2023/subsystems/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : frc846::Subsystem<LEDsReadings, LEDsTarget>("leds") {
  leds_.SetLength(kHumanLength + kHumanLength1 + 
                  kHumanLength2 + kDriverLength1 + 
                  kDriverLength2);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

LEDsTarget LEDsSubsystem::ZeroTarget() const {
  LEDsTarget target;
  target.human_player = kCube;
  target.driver = kNoPiece;
  target.has_zeroed = true;
  return target;
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::GetNewReadings() { return {}; }

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  if (!target.has_zeroed) {
    loops++;
    if (loops % 100 > 50) {
      for (int i = 0; i < (kHumanLength + kHumanLength1 + kDriverLength1 + kDriverLength2 + kHumanLength2); i++) {
        leds_buffer_[i].SetRGB(255, 0, 0);
      }
    } else {
      for (int i = 0; i < (kHumanLength + kHumanLength1 + kDriverLength1 + kDriverLength2 + kHumanLength2); i++) {
        leds_buffer_[i].SetRGB(0, 255, 0);
      }
    }
  } else if (target.human_player == LEDsState::kCone) {
      for (int i = 0; i < kHumanLength + kHumanLength1; i++) {
        leds_buffer_[i].SetRGB(246, 190, 0);
      }
      for (int i = kHumanLength + kHumanLength1 + kDriverLength1; i < kHumanLength + kHumanLength1 + kDriverLength1 + kHumanLength2; i++){
        leds_buffer_[i].SetRGB(246, 190, 0);
      }
    } else if (target.human_player == LEDsState::kCube) {
      for (int i = 0; i < kHumanLength + kHumanLength1; i++) {
        leds_buffer_[i].SetRGB(75, 0, 130);
      }
      for (int i = kHumanLength + kHumanLength1 + kDriverLength1; i < kHumanLength + kHumanLength1 + kDriverLength1 + kHumanLength2; i++){
        leds_buffer_[i].SetRGB(75, 0, 130);
      }
    }

    if (target.driver == LEDsState::kNoPiece) {
      for (int i = kHumanLength + kHumanLength1 ; i < (kHumanLength + kHumanLength1 + kDriverLength1); i++) {
        leds_buffer_[i].SetRGB(0, 255, 0);
      }
      for (int i = kHumanLength + kHumanLength1 + kDriverLength1 + kDriverLength2 ; i < (kHumanLength + kHumanLength1 + kDriverLength1 + kDriverLength2 + kHumanLength2); i++) {
        leds_buffer_[i].SetRGB(0, 255, 0);
      }
    } else if (target.driver == LEDsState::kHasPiece) {
      for (int i = kHumanLength + kHumanLength1; i < (kHumanLength + kHumanLength1 + kDriverLength1); i++) {
        leds_buffer_[i].SetRGB(0, 255, 130);
      }
      for (int i = kHumanLength + kHumanLength1 + kDriverLength1 + kDriverLength2 ; i < (kHumanLength + kHumanLength1 + kDriverLength1 + kDriverLength2 + kHumanLength2); i++) {
        leds_buffer_[i].SetRGB(0, 255, 130);
      }
    }
  leds_.SetData(leds_buffer_);

}

  
  

