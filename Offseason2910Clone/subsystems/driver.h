#ifndef Offseason2910Clone_SUBSYSTEMS_DRIVER_H_
#define Offseason2910Clone_SUBSYSTEMS_DRIVER_H_

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/grapher.h"
#include "transitionLib846/math.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"
#include "transitionLib846/xbox.h"


using DriverReadings = transitionLib846::XboxReadings;

struct DriverTarget {
  bool rumble;
};

class DriverSubsystem
    : public transitionLib846::Subsystem<DriverReadings, DriverTarget> {
 public:
  DriverSubsystem();

  transitionLib846::Pref<double> translation_deadband_{
      *this, "translation_deadband", 0.05};
  transitionLib846::Pref<double> steer_deadband_{*this, "steer_deadband", 0.05};

  transitionLib846::Pref<int> translation_exponent_{*this,
                                                    "translation_exponent", 1};
  transitionLib846::Pref<int> steer_exponent_{*this, "steer_exponent", 2};

  DriverTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  transitionLib846::Pref<double> trigger_threshold_{*this, "trigger_threshold",
                                                    0.3};
  transitionLib846::Pref<double> rumble_strength_{*this, "rumble_strength",
                                                  1.0};

  frc::XboxController xbox_{ports::driver::kXbox_DSPort};

  transitionLib846::Named target_named_{*this, "target"};
  transitionLib846::Grapher<bool> target_rumble_graph_{target_named_, "rumble"};

  DriverReadings GetNewReadings() override;

  void WriteToHardware(DriverTarget target) override;
};

#endif  // Offseason2910Clone_SUBSYSTEMS_DRIVER_H_