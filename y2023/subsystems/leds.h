#ifndef Y2023_SUBSYSTEMS_LEDS_H_
#define Y2023_SUBSYSTEMS_LEDS_H_

#include <frc/AddressableLED.h>

#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "y2023/ports.h"

struct LEDsReadings {};

enum LEDsState {
  kCone,
  kCube,
  kHasPiece,
  kNoPiece,
};

struct LEDsTarget {
  LEDsState human_player;
  LEDsState driver;
  bool has_zeroed;
};

class LEDsSubsystem : public frc846::Subsystem<LEDsReadings, LEDsTarget> {
 public:
  LEDsSubsystem();

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Number of LEDs.
  static constexpr int kHumanLength = 33;
  static constexpr int kHumanLength1 = 7;
  static constexpr int kHumanLength2 = 7;
  static constexpr int kDriverLength1 = 7;
  static constexpr int kDriverLength2 = 7;

  std::array<frc::AddressableLED::LEDData, kHumanLength + kHumanLength1 + 
                                          kHumanLength2 + kDriverLength1 + 
                                          kDriverLength2> leds_buffer_;

  frc::AddressableLED leds_{ports::leds::kPWMPort};

  LEDsReadings GetNewReadings() override;

  int loops;

  void WriteToHardware(LEDsTarget target) override;
};

using OptionalLEDsSubsystem =
    frc846::OptionalSubsystem<LEDsSubsystem, LEDsReadings, LEDsTarget>;

#endif  // Y2023_SUBSYSTEMS_LEDS_H_