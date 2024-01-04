#ifndef Y2023_SUBSYSTEMS_CONVEYOR_H_
#define Y2023_SUBSYSTEMS_CONVEYOR_H_

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/Talon.h>
#include <rev/ColorSensorV3.h>

#include "frc846/conversions.h"
#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "y2023/ports.h"

enum class GamePieceState { kNone, kCube, kCone };

struct ConveyorReadings {
  GamePieceState game_piece;
  bool is_color_connected;
  bool transfer_ready;
};

struct ConveyorTarget {
  double left_roller_speed;
  double right_roller_speed;
  double belt_speed;
};

class ConveyorSubsystem
    : public frc846::Subsystem<ConveyorReadings, ConveyorTarget> {
 public:
  ConveyorSubsystem();

  ConveyorTarget ZeroTarget() const override;
  bool VerifyHardware() override;

  frc846::Named speeds_named_{*this, "speeds"};
  // default speed
  frc846::Pref<double> default_roller_speed_{speeds_named_,
                                             "default_roller_speed", 0.7};

  frc846::Pref<double> reverse_roller_speed_{speeds_named_,
                                             "reverse_roller_speed", -0.7};

  frc846::Pref<double> belt_speed_{speeds_named_, "belt_speed", 0.5};
  frc846::Pref<double> differential_speed_{speeds_named_, "differential_speed",
                                           1.0};
  frc846::Pref<double> reverse_belt_speed_{speeds_named_, "reverse_belt_speed",
                                           -0.7};

  frc846::Pref<bool> should_run_{*this, "should_run", true};

 private:
  int loops;
  frc::Talon left_roller_esc_{ports::conveyor::kLeftRoller_PWM};
  frc::Talon right_roller_esc_{ports::conveyor::kRightRoller_PWM};

  frc::Talon belt_esc_{ports::conveyor::kBelt_PWM};

  ConveyorReadings GetNewReadings() override;
  void WriteToHardware(ConveyorTarget target) override;
};

using OptionalConveyorSubsystem =
    frc846::OptionalSubsystem<ConveyorSubsystem, ConveyorReadings,
                              ConveyorTarget>;

#endif  // Y2023_SUBSYSTEMS_CONVEYOR_H_