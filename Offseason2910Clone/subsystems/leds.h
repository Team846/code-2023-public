#ifndef Offseason2910Clone_SUBSYSTEMS_LEDS_H_
#define Offseason2910Clone_SUBSYSTEMS_LEDS_H_

#include <frc/AddressableLED.h>

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"


struct LEDsReadings {};

enum LEDsState {
  kCone,
  kCube,
  kNotZeroed,
};

struct LEDsTarget {
  LEDsState state;
  bool has_zeroed;
};

class LEDsSubsystem
    : public transitionLib846::Subsystem<LEDsReadings, LEDsTarget> {
 public:
  LEDsSubsystem();

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Number of LEDs.
  static constexpr int kLength = 33;

  std::array<frc::AddressableLED::LEDData, kLength> leds_buffer_;

  frc::AddressableLED leds_{ports::leds::kPWMPort};

  LEDsReadings GetNewReadings() override;

  int loops;

  void WriteToHardware(LEDsTarget target) override;
};

using OptionalLEDsSubsystem =
    transitionLib846::OptionalSubsystem<LEDsSubsystem, LEDsReadings,
                                        LEDsTarget>;

#endif  // Offseason2910Clone_SUBSYSTEMS_LEDS_H_