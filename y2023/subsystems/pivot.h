#ifndef Y2023_SUBSYSTEMS_PIVOT_H_
#define Y2023_SUBSYSTEMS_PIVOT_H_

#include <frc/DigitalInput.h>
#include <units/angle.h>

#include "frc846/conversions.h"
#include "frc846/grapher.h"
#include "frc846/motor/helper.h"
#include "frc846/named.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "y2023/ports.h"

struct PivotReadings {
  units::volt_t raw_pot;
  units::degree_t pot_position;
  units::degree_t position;
};

using PivotPosition = units::degree_t;
using DutyCycle = double;

struct PivotTarget {
  std::variant<PivotPosition, DutyCycle> output;
};

class PivotSubsystem : public frc846::Subsystem<PivotReadings, PivotTarget> {
 public:
  PivotSubsystem();

  // Named up & down so there isn't confusion with telescoping
  frc846::Named speeds_named_{*this, "pivot_speeds"};
  frc846::Pref<double> up_speed_{speeds_named_, "up_speed", 0.1};
  frc846::Pref<double> down_speed_{speeds_named_, "down_speed", -0.2};

  frc846::Named positions_named_{*this, "pivot_position"};

  frc846::Pref<units::degree_t> retract_tolerance_{positions_named_,
                                                   "retract_tolerance", 25_deg};
  frc846::Pref<units::degree_t> min_pivot_{positions_named_, "min_pivot",
                                           0_deg};
  frc846::Pref<units::degree_t> retract_position_{positions_named_, "retract",
                                                  0_deg};
  frc846::Pref<units::degree_t> min_conveyor_{positions_named_, "min_conveyor",
                                              0_deg};
  frc846::Pref<units::degree_t> place_position_{positions_named_,
                                                "place_position", 20_deg};

  frc846::Pref<units::degree_t> position_tolerance_{
      positions_named_, "position_tolerance", 0.5_deg};

  // CONE
  frc846::Named cone_named_{positions_named_, "cone"};
  frc846::Pref<units::degree_t> cone_mid_position_{cone_named_, "cone_mid",
                                                   83_deg};
  frc846::Pref<units::degree_t> cone_high_position_{cone_named_, "cone_high",
                                                    106.5_deg};
  frc846::Pref<units::degree_t> cone_shelf_position_{cone_named_, "cone_shelf",
                                                     81.5_deg};
  frc846::Pref<units::degree_t> cone_conveyor_position_{
      cone_named_, "cone_conveyor", -8.5_deg};

  frc846::Pref<units::degree_t> hybrid_position_{positions_named_, "hybrid",
                                                 27_deg};

  // How much you have to pivot before extending telescope
  frc846::Pref<units::degree_t> cone_min_mid_{cone_named_, "cone_min_mid",
                                              83_deg};
  frc846::Pref<units::degree_t> cone_min_high_{cone_named_, "cone_min_high",
                                               84_deg};

  // CUBE
  frc846::Named cube_named_{positions_named_, "cube"};
  frc846::Pref<units::degree_t> cube_mid_position_{cube_named_, "cube_mid",
                                                   70_deg};
  frc846::Pref<units::degree_t> cube_high_position_{cube_named_, "cube_high",
                                                    94_deg};
  frc846::Pref<units::degree_t> cube_shelf_position_{cube_named_, "cube_shelf",
                                                     85.2_deg};
  frc846::Pref<units::degree_t> cube_conveyor_position_{
      cube_named_, "cube_conveyor", -9.5_deg};

  // How much you have to pivot before extending telescope
  frc846::Pref<units::degree_t> cube_min_mid_{cube_named_, "cube_min_mid",
                                              70_deg};
  frc846::Pref<units::degree_t> cube_min_high_{cube_named_, "cube_min_high",
                                               84_deg};

  void Zero();

  PivotTarget HoldPosition();

  PivotTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  static constexpr units::turn_t kPotRange = 3_tr;
  // Min limit for pot.
  static constexpr units::volt_t kMinLimit = 1.735_V;
  // Max limit for pot.
  static constexpr units::volt_t kMaxLimit = 2.9_V;

  static constexpr units::volt_t kLowerPotVoltage = 0.02_V;
  static constexpr units::volt_t kPotVoltRange = 3.324_V - kLowerPotVoltage;

  // Ratio from pot to arm pivot (sprocket only)
  static constexpr double kSprocketRatio = 74.0 / 24.0;

  static constexpr bool kInvertedPot = true;

  frc846::Pref<units::degree_t> potentiometer_zero_pivot_position_{
      *this, "pot_zero_position", 0_deg};
  frc846::Pref<units::volt_t> potentiometer_zero_voltage_{
      *this, "pot_zero_voltage", 1.825_V};

  frc846::Grapher<units::volt_t> pot_voltage_graph_{*this, "pot_voltage"};
  frc846::Grapher<units::degree_t> pot_position_graph_{*this, "pot_position"};

  frc846::Grapher<units::degree_t> position_graph_{*this, "position"};
  frc846::Grapher<units::degree_t> target_position_graph_{*this,
                                                          "target_position"};

  frc846::Named esc_named_{*this, "esc"};

  rev::CANSparkMax esc_{
      ports::pivot::kPivot_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  frc846::motor::GainsHelper* esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          esc_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  frc846::motor::SparkMAXHelper esc_helper_{
      esc_named_,
      esc_,
      new frc846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_gains_helper_,
  };

  frc846::Converter<units::degree_t> converter_{
      frc846::kSparkMAXPeriod,
      frc846::kSparkMAXSensorTicks,
      // 2x 5:1 MAXPlanetary, 2:1 gearstage, 74:24 sprocket
      1_tr / ((5 / 1) * (5 / 1) * (2 / 1) * (74 / 24)),
  };

  rev::SparkMaxAnalogSensor potentiometer_{esc_.GetAnalog()};
  PivotReadings GetNewReadings() override;
  void WriteToHardware(PivotTarget target) override;
};

using OptionalPivotSubsystem =
    frc846::OptionalSubsystem<PivotSubsystem, PivotReadings, PivotTarget>;

#endif  //  Y2023_SUBSYSTEMS_PIVOT_H_