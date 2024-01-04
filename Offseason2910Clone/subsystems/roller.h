#ifndef Offseason2910Clone_SUBSYSTEMS_ROLLER_H_
#define Offseason2910Clone_SUBSYSTEMS_ROLLER_H_

#include <units/length.h>

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/conversions.h"
#include "transitionLib846/current_sensor.h"
#include "transitionLib846/grapher.h"
#include "transitionLib846/motor/config.h"
#include "transitionLib846/motor/helper.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"

enum GamePieces {
  kGrab,
  kRelease,
  kEmpty,
  kHold,
};

struct RollerReadings {
  bool has_piece;
};

struct RollerTarget {
  GamePieces state;
};

class RollerSubsystem
    : public transitionLib846::Subsystem<RollerReadings, RollerTarget> {
 public:
  bool cube_mode = false;

  GamePieces wanted_state = kEmpty;

  RollerSubsystem();

  // TODO: Find ideal current spike value
  transitionLib846::Pref<units::ampere_t> current_threshold_{
      *this, "current_threshold", 7.5_A};
  transitionLib846::Pref<double> speed_threshold_cone_{
      *this, "speed_threshold_cone", 10};
  transitionLib846::Pref<double> speed_threshold_cube_{
      *this, "speed_threshold_cube", 6};

  transitionLib846::Named speeds_named_{*this, "speeds"};
  // Speed when picking up for cones
  transitionLib846::Pref<double> grab_cone_speed_{speeds_named_,
                                                  "grab_cone_speed", 0.2};
  // Speed when picking up for cubes
  transitionLib846::Pref<double> grab_cube_speed_{speeds_named_,
                                                  "grab_cube_speed", -0.2};
  // Speed when placing for cones
  transitionLib846::Pref<double> place_cone_speed_{speeds_named_,
                                                   "place_cone_speed", -0.3};
  // Speed when placing for cubes
  transitionLib846::Pref<double> place_cube_speed_{speeds_named_,
                                                   "place_cube_speed", 0.3};

  transitionLib846::Pref<double> idle_speed_cone_{speeds_named_,
                                                  "idle_speed_cone", -0.05};
  transitionLib846::Pref<double> idle_speed_cube_{speeds_named_,
                                                  "idle_speed_cube", -0.05};
  transitionLib846::Pref<bool> should_auto_pickup_{*this, "should_auto_pickup",
                                                   true};

  transitionLib846::Converter<units::inch_t> converter_{
      transitionLib846::kSparkMAXPeriod, transitionLib846::kSparkMAXSensorTicks,
      1_in};

  RollerTarget ZeroTarget() const override;
  RollerTarget GrabTarget();
  RollerTarget ReleaseTarget();
  bool VerifyHardware() override;
  void ToggleCubeMode();
  void ToggleCubeMode(bool mode);
  bool GetCubeMode();

 private:
  transitionLib846::Grapher<units::ampere_t> current_graph_{*this,
                                                            "roller_current"};
  transitionLib846::Grapher<units::ampere_t> avg_current_graph_{
      *this, "avg_roller_current"};
  transitionLib846::Grapher<bool> has_piece_graph_{*this, "roller_has_piece"};

  rev::CANSparkMax esc_{
      ports::roller::kCANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  transitionLib846::Named target_named_{*this, "target"};
  transitionLib846::Grapher<double> target_speed_graph_{target_named_, "speed"};

  transitionLib846::Named esc_named_{*this, "esc"};

  // TODO: Update Curent limits
  transitionLib846::motor::SparkMAXHelper esc_helper_{
      esc_named_,
      esc_,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,    // peak output
              12_V,   // voltage comp staturation
              200_A,  // current limit
          },
      },
      nullptr,
  };

  transitionLib846::CurrentSensor roller_current_sensor{current_threshold_};
  int loops_since_ = 0;
  bool has_piece_ = false;

  double last_roller_speed_ = 0.0;

  RollerReadings GetNewReadings() override;

  void WriteToHardware(RollerTarget target) override;
};

using OptionalRollerSubsystem =
    transitionLib846::OptionalSubsystem<RollerSubsystem, RollerReadings,
                                        RollerTarget>;

#endif  // Offseason2910Clone_SUBSYSTEMS_Roller_H_