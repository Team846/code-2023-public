#ifndef Y2023_SUBSYSTEMS_GRIPPER_H_
#define Y2023_SUBSYSTEMS_GRIPPER_H_

#include "frc846/current_sensor.h"
#include "frc846/grapher.h"
#include "frc846/motor/config.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "y2023/ports.h"

struct GripperReadings {
  bool has_piece;
};

struct GripperTarget {
  double speed;  // Output is [-1, 1]
};

class GripperSubsystem
    : public frc846::Subsystem<GripperReadings, GripperTarget> {
 public:
  GripperSubsystem();

  // TODO: Find ideal current spike value
  frc846::Pref<units::ampere_t> current_threshold_{*this, "current_threshold",
                                                   7.5_A};

  frc846::Named speeds_named_{*this, "speeds"};
  // Speed when picking up
  frc846::Pref<double> grab_speed_{speeds_named_, "grab_speed", -0.65};
  // Speed when placing
  frc846::Pref<double> place_speed_{speeds_named_, "place_speed", 0.9};
  // Default Speed
  frc846::Pref<double> reset_target_speed_{speeds_named_, "reset_target_speed", -2.0};

  frc846::Pref<double> idle_speed_{speeds_named_, "idle_speed", -0.10};
  frc846::Pref<bool> should_auto_pickup_{*this, "should_auto_pickup", true};

  GripperTarget ZeroTarget() const override;
  bool VerifyHardware() override;

 private:
  frc846::Grapher<units::ampere_t> current_graph_{*this, "roller_current"};
  frc846::Grapher<units::ampere_t> avg_current_graph_{*this,
                                                      "avg_roller_current"};
  frc846::Grapher<bool> has_piece_graph_{*this, "gripper_has_piece"};

  ctre::TalonSRX esc_{ports::gripper::kCANID};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<double> target_speed_graph_{target_named_, "speed"};

  frc846::Named esc_named_{*this, "esc"};

  // TODO: Update Curent limits
  frc846::motor::TalonSRXHelper esc_helper_{
      esc_named_,
      esc_,
      new frc846::motor::TalonSRXConfigHelper{
          esc_named_,
          {
              1.0,    // peak output
              12_V,   // voltage comp staturation
              10_A,   // peak current limit
              0.5_s,  // peak current duration
              4_A,    // continuous current limit
          },
      },
      nullptr,
  };

  frc846::CurrentSensor roller_current_sensor{current_threshold_};
  int loops_since_ = 0;
  bool has_piece_ = false;
  int kLoops = 0;

  GripperReadings GetNewReadings() override;

  void WriteToHardware(GripperTarget target) override;
};

using OptionalGripperSubsystem =
    frc846::OptionalSubsystem<GripperSubsystem, GripperReadings, GripperTarget>;

#endif  // Y2023_SUBSYSTEMS_GRIPPER_H_