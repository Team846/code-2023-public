#ifndef Offseason2910Clone_SUBSYSTEMS_PIVOT_H_
#define Offseason2910Clone_SUBSYSTEMS_PIVOT_H_

#include <frc/DigitalInput.h>
#include <units/angle.h>

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/conversions.h"
#include "transitionLib846/grapher.h"
#include "transitionLib846/motor/helper.h"
#include "transitionLib846/named.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"


struct PivotReadings {
  units::degree_t position;
};

using PivotPosition = units::degree_t;
using DutyCycle = double;

struct PivotTarget {
  std::variant<PivotPosition, DutyCycle> output;
};

class PivotSubsystem
    : public transitionLib846::Subsystem<PivotReadings, PivotTarget> {
 public:
  PivotSubsystem();

  // Named up & down so there isn't confusion with telescoping
  transitionLib846::Named speeds_named_{*this, "pivot_speeds"};
  transitionLib846::Pref<double> up_speed_{speeds_named_, "up_speed", 0.1};
  transitionLib846::Pref<double> down_speed_{speeds_named_, "down_speed", -0.2};

  void ZeroPosition(bool zeroed);

  PivotTarget HoldPosition();
  PivotTarget ToPosition(transitionLib846::Pref<units::degree_t> pos);
  PivotTarget PlacePosition();

  PivotTarget ZeroTarget() const override;
  PivotTarget UpTarget() const;
  PivotTarget DownTarget() const;

  void MakeAllCoastMode();
  void MakeAllBrakeMode();

  bool VerifyHardware() override;

  transitionLib846::Pref<double> ff_multiplier_{*this, "ff_multiplier", 0.0};
  transitionLib846::Pref<double> p_gain_{*this, "p_gain", 0.0};

  transitionLib846::Named positions_named{*this, "pivot_position"};
  transitionLib846::Pref<units::degree_t> place_amount_{positions_named,
                                                        "place_amount", 10_deg};

  transitionLib846::Pref<units::degree_t> cone_high_{positions_named,
                                                     "cone_high_", 10_deg};

  transitionLib846::Pref<units::degree_t> cone_mid_{positions_named,
                                                    "cone_mid_", 10_deg};

  transitionLib846::Pref<units::degree_t> cone_shelf_{positions_named,
                                                      "cone_shelf", 10_deg};

  transitionLib846::Pref<units::degree_t> cone_shelf_min_{
      positions_named, "cone_shelf_min", 10_deg};

  transitionLib846::Pref<units::degree_t> cube_high_{positions_named,
                                                     "cube_high", 10_deg};

  transitionLib846::Pref<units::degree_t> cube_mid_{positions_named, "cube_mid",
                                                    10_deg};

  transitionLib846::Pref<units::degree_t> cube_shelf_{positions_named,
                                                      "cube_shelf", 10_deg};

  transitionLib846::Pref<units::degree_t> cube_shelf_min_{
      positions_named, "cube_shelf_min", 10_deg};

  transitionLib846::Pref<units::degree_t> cube_single_{positions_named,
                                                       "cube_single", 10_deg};
  transitionLib846::Pref<units::degree_t> cone_single_{positions_named,
                                                       "cone_single", 10_deg};

  transitionLib846::Pref<units::degree_t> hybrid_cone_{positions_named,
                                                       "hybrid_cone", 10_deg};
  transitionLib846::Pref<units::degree_t> hybrid_cube_{positions_named,
                                                       "hybrid_cube", 10_deg};

  transitionLib846::Pref<units::degree_t> stow_position_{
      positions_named, "stow_position", 0_deg};

  transitionLib846::Pref<units::degree_t> cone_intake_low_{
      positions_named, "cone_intake_low", 10_deg};

  transitionLib846::Pref<units::degree_t> cube_intake_low_{
      positions_named, "cube_intake_low", 10_deg};

 private:
  double custom_pid(double err, double min, double max);

  bool has_zeroed;

  // Ratio from motor -> pivot
  static constexpr double kReduction = 1;  // TODO: change this

  transitionLib846::Grapher<units::degree_t> position_graph_{*this, "position"};
  transitionLib846::Grapher<units::degree_t> target_position_graph_{
      *this, "target_position"};

  transitionLib846::Named esc_l1_named_{*this, "esc_l1"};

  rev::CANSparkMax esc_l1{
      ports::pivot::kL1_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  transitionLib846::motor::GainsHelper* esc_l1_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_l1_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  transitionLib846::motor::SparkMAXHelper esc_l1_helper_{
      esc_l1_named_,
      esc_l1,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_l1_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_l1_gains_helper_,
  };

  transitionLib846::Named esc_l2_named_{*this, "esc_l2"};

  rev::CANSparkMax esc_l2{
      ports::pivot::kL2_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  transitionLib846::motor::GainsHelper* esc_l2_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_l2_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  transitionLib846::motor::SparkMAXHelper esc_l2_helper_{
      esc_l2_named_,
      esc_l2,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_l2_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_l2_gains_helper_,
  };

  transitionLib846::Named esc_r1_named_{*this, "esc_r1"};

  rev::CANSparkMax esc_r1{
      ports::pivot::kR1_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  transitionLib846::motor::GainsHelper* esc_r1_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_r1_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  transitionLib846::motor::SparkMAXHelper esc_r1_helper_{
      esc_r1_named_,
      esc_r1,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_r1_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_r1_gains_helper_,
  };

  transitionLib846::Named esc_r2_named_{*this, "esc_r2"};

  rev::CANSparkMax esc_r2{
      ports::pivot::kR2_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  transitionLib846::motor::GainsHelper* esc_r2_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_r2_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  transitionLib846::motor::SparkMAXHelper esc_r2_helper_{
      esc_r2_named_,
      esc_r2,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_r2_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_r2_gains_helper_,
  };

  transitionLib846::Converter<units::degree_t> converter_{
      transitionLib846::kSparkMAXPeriod,
      transitionLib846::kSparkMAXSensorTicks,
      1_tr / 110,
  };

  PivotReadings GetNewReadings() override;
  void WriteToHardware(PivotTarget target) override;
};

using OptionalPivotSubsystem =
    transitionLib846::OptionalSubsystem<PivotSubsystem, PivotReadings,
                                        PivotTarget>;

#endif  //  Offseason2910Clone_SUBSYSTEMS_PIVOT_H_