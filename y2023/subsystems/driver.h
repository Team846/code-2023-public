#ifndef Y2023_SUBSYSTEMS_DRIVER_H_
#define Y2023_SUBSYSTEMS_DRIVER_H_

#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "frc846/xbox.h"
#include "y2023/ports.h"

using DriverReadings = frc846::XboxReadings;

struct DriverTarget {
  bool rumble;
};

class DriverSubsystem : public frc846::Subsystem<DriverReadings, DriverTarget> {
 public:
  DriverSubsystem();

  frc846::Pref<double> translation_deadband_{*this, "translation_deadband",
                                             0.05};
  frc846::Pref<double> steer_deadband_{*this, "steer_deadband", 0.05};

  frc846::Pref<int> translation_exponent_{*this, "translation_exponent", 1};
  frc846::Pref<int> steer_exponent_{*this, "steer_exponent", 2};

  DriverTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Pref<double> trigger_threshold_{*this, "trigger_threshold", 0.3};
  frc846::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::driver::kXbox_DSPort};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<bool> target_rumble_graph_{target_named_, "rumble"};

  DriverReadings GetNewReadings() override;

  void WriteToHardware(DriverTarget target) override;
};

#endif  // Y2023_SUBSYSTEMS_DRIVER_H_