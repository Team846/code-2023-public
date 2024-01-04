#ifndef Y2023_SUBSYSTEMS_WRIST_H_
#define Y2023_SUBSYSTEMS_WRIST_H_

#include "frc846/grapher.h"
#include "frc846/motor/config.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "y2023/ports.h"

enum WristState { kForward, kBackward };

struct WristReadings {};

struct WristTarget {
  double speed;
};

class WristSubsystem : public frc846::Subsystem<WristReadings, WristTarget> {
 public:
  WristSubsystem();

  frc846::Named speeds_named_{*this, "speeds"};
  // Speed when picking up
  frc846::Pref<double> flip_speed_{speeds_named_, "flip_speed", 0.5};
  // Speed when placing
  frc846::Pref<double> unflip_speed_{speeds_named_, "unflip_speed", -0.5};
  // Hold Voltage
  frc846::Pref<double> hold_speed_{speeds_named_, "hold_speed", 0.1};

  bool prev_state_ = kBackward;
  WristTarget ZeroTarget() const override;
  bool VerifyHardware() override;

 private:
  ctre::TalonSRX esc_{ports::wrist::kCANID};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<double> target_speed_graph_{target_named_, "speed"};

  frc846::Named esc_named_{*this, "esc"};

  frc846::motor::TalonSRXHelper esc_helper_{
      esc_named_,
      esc_,
      new frc846::motor::TalonSRXConfigHelper{
          esc_named_,
          {
              1.0,    // peak output
              12_V,   // voltage comp staturation
              6_A,    // peak current limit
              0.5_s,  // peak current duration
              2_A,    // continuous current limit
          },
      },
      nullptr,
  };

  WristReadings GetNewReadings() override;

  void WriteToHardware(WristTarget target) override;
};

using OptionalWristSubsystem =
    frc846::OptionalSubsystem<WristSubsystem, WristReadings, WristTarget>;

#endif  // Y2023_SUBSYSTEMS_WRIST_H_