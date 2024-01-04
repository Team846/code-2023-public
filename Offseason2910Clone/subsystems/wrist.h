#ifndef Offseason2910Clone_SUBSYSTEMS_WRIST_H_
#define Offseason2910Clone_SUBSYSTEMS_WRIST_H_

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/conversions.h"
#include "transitionLib846/grapher.h"
#include "transitionLib846/motor/config.h"
#include "transitionLib846/motor/helper.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"
#include "units/math.h"

struct WristReadings {
  units::degree_t position;
};

using WristPosition = units::degree_t;
using DutyCycle = double;

struct WristTarget {
  std::variant<WristPosition, DutyCycle> output;
};

class WristSubsystem
    : public transitionLib846::Subsystem<WristReadings, WristTarget> {
 public:
  WristSubsystem();

  WristTarget ZeroTarget() const override;
  WristTarget ToPosition(transitionLib846::Pref<units::degree_t> pos);
  bool VerifyHardware() override;
  void SetHasHomed(bool homed);
  void ResetPosition();

  units::degree_t prev_target = 0_deg;

  WristTarget HoldPosition();
  WristTarget IncrementPosition(units::degree_t increment);

  transitionLib846::Named positions_named_{*this, "positions"};

  transitionLib846::Pref<units::degree_t> cone_intake_position_{
      positions_named_, "cone_intake_position", 100_deg};

  transitionLib846::Pref<units::degree_t> cube_intake_position_{
      positions_named_, "cube_intake_position", 40_deg};
  transitionLib846::Pref<units::degree_t> cone_high_position_{
      positions_named_, "cone_place_position", 100_deg};

  transitionLib846::Pref<units::degree_t> cone_mid_position_{
      positions_named_, "cone_place_position", 100_deg};

  transitionLib846::Pref<units::degree_t> cone_low_position_{
      positions_named_, "cone_place_position", 100_deg};

  transitionLib846::Pref<units::degree_t> cube_low_position_{
      positions_named_, "cube_low_position", 40_deg};

  transitionLib846::Pref<units::degree_t> cube_place_position_{
      positions_named_, "cube_place_position", 40_deg};

  transitionLib846::Pref<units::degree_t> cone_shelf_position_{
      positions_named_, "cone_shelf_position", 100_deg};

  transitionLib846::Pref<units::degree_t> cube_shelf_position_{
      positions_named_, "cube_shelf_position", 40_deg};

  transitionLib846::Pref<units::degree_t> cube_single_position_{
      positions_named_, "cube_single_position", 100_deg};

  transitionLib846::Pref<units::degree_t> cone_single_position_{
      positions_named_, "cone_single_position", 40_deg};

  transitionLib846::Pref<units::degree_t> idle_position_{
      positions_named_, "idle_position", 0_deg};

 private:
  static constexpr double gearRatio = 0.0293;

  transitionLib846::Grapher<units::degree_t> position_graph_{*this, "position"};
  transitionLib846::Grapher<units::degree_t> target_position_graph_{
      *this, "target_position"};

  transitionLib846::Grapher<bool> has_homed_graph_{*this, "has_homed"};

  bool has_homed_ = false;

  rev::CANSparkMax esc_{ports::wrist::kLeft_CANID,
                        rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  transitionLib846::Named esc_named_{*this, "left_esc"};

  transitionLib846::motor::GainsHelper* esc_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_named_,
          {
              0.03, /* p */
              0,    /* i */
              0.7,  /* d */
              0,    /* f */
              0,    /* max_integral_accumulator */
          },
      };  // change values TODO

  transitionLib846::motor::SparkMAXHelper esc_helper_{
      esc_named_, esc_,
      // TODO update
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,   // peak output
              10_V,  // voltage compensation
              20_A,  // current limit
          },
      },
      esc_gains_helper_};

  // TODO change ratio
  transitionLib846::Converter<units::degree_t> converter_{
      transitionLib846::kSparkMAXPeriod, transitionLib846::kSparkMAXSensorTicks,
      gearRatio * (340_deg)};

  WristReadings GetNewReadings() override;

  void WriteToHardware(WristTarget target) override;
};

using OptionalWristSubsystem =
    transitionLib846::OptionalSubsystem<WristSubsystem, WristReadings,
                                        WristTarget>;

#endif  // Offseason2910Clone_SUBSYSTEMS_Wrist_H_