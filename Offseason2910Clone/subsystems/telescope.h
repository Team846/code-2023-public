#ifndef Y2023_SUBSYSTEMS_TELESCOPE_H_
#define Y2023_SUBSYSTEMS_TELESCOPE_H_

#include <frc/DigitalInput.h>
#include <units/length.h>

#include "Offseason2910Clone/ports.h"
#include "transitionLib846/conversions.h"
#include "transitionLib846/grapher.h"
#include "transitionLib846/motor/helper.h"
#include "transitionLib846/named.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"

struct TelescopeReadings {
  bool limit_reached;
  units::inch_t extension;
};

using TelescopePosition = units::inch_t;
using DutyCycle = double;

struct TelescopeTarget {
  std::variant<TelescopePosition, DutyCycle> output;
};

class TelescopeSubsystem
    : public transitionLib846::Subsystem<TelescopeReadings, TelescopeTarget> {
 public:
  TelescopeSubsystem();
  // TODO figure out all the numbers
  // Named out & in so there isn't confusion with pivot
  transitionLib846::Named speeds_named_{*this, "telescope_speeds"};
  transitionLib846::Pref<double> out_speed_{speeds_named_, "in_speed", 0.0};
  transitionLib846::Pref<double> in_speed_{speeds_named_, "out_speed", 0.0};

  transitionLib846::Named positions_named_{*this, "telescope_position"};

  transitionLib846::Pref<units::inch_t> retract_tolerance_{
      positions_named_, "retract_tolerance", 0.5_in};

  transitionLib846::Pref<units::inch_t> min_extension_{positions_named_,
                                                       "min_extension", 0_in};
  transitionLib846::Pref<units::inch_t> retract_position_{positions_named_,
                                                          "retract", 0_in};
  transitionLib846::Pref<units::inch_t> place_position_{positions_named_,
                                                        "place_position", 0_in};

  transitionLib846::Pref<units::inch_t> position_tolerance_{
      positions_named_, "position_tolerance", 0.5_in};

  // CONE
  transitionLib846::Named cone_named_{positions_named_, "cone"};
  transitionLib846::Pref<units::inch_t> cone_mid_position_{cone_named_,
                                                           "cone_mid", 0_in};
  transitionLib846::Pref<units::inch_t> cone_high_position_{cone_named_,
                                                            "cone_high", 0_in};
  transitionLib846::Pref<units::inch_t> cone_shelf_position_{
      cone_named_, "cone_shelf", 0_in};
  transitionLib846::Pref<units::inch_t> cone_single_position_{
      cone_named_, "cone_single", 0_in};
  transitionLib846::Pref<units::inch_t> cone_intake_position_{
      cone_named_, "cone_intake", 0_in};

  transitionLib846::Pref<units::inch_t> hybrid_position_{positions_named_,
                                                         "hybrid", 0_in};

  // CUBE
  transitionLib846::Named cube_named_{positions_named_, "cube"};
  transitionLib846::Pref<units::inch_t> cube_mid_position_{cube_named_,
                                                           "cube_mid", 0_in};
  transitionLib846::Pref<units::inch_t> cube_high_position_{cube_named_,
                                                            "cube_high", 0_in};
  transitionLib846::Pref<units::inch_t> cube_shelf_position_{
      cube_named_, "cube_shelf", 0_in};
  transitionLib846::Pref<units::inch_t> cube_intake_position_{
      cube_named_, "cube_intake", 0_in};
  transitionLib846::Pref<units::inch_t> cube_single_position_{
      cube_named_, "cube_single", 0_in};

  void SetHasZeroed(bool zeroed);

  TelescopeTarget HoldPosition();
  TelescopeTarget OutTarget() const;
  TelescopeTarget InTarget() const;
  TelescopeTarget ToPosition(units::inch_t pos);
  TelescopeTarget ToPosition(transitionLib846::Pref<units::inch_t> pos);
  void MakeAllCoast();
  void MakeAllBrake();

  TelescopeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

  bool has_zeroed;

 private:
  // Ratio from motor -> telescope
  static constexpr double kReduction = 1;  // TODO: change this

  transitionLib846::Grapher<units::inch_t> position_graph_{*this, "position"};
  transitionLib846::Grapher<units::inch_t> target_position_graph_{
      *this, "target_position"};

  transitionLib846::Named esc_tele1_named_{*this, "esc_tele1"};

  rev::CANSparkMax esc_tele1{
      ports::telescope::kTele1_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };
  // TODO: Change PIDF
  transitionLib846::motor::GainsHelper* esc_tele1_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_tele1_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  transitionLib846::motor::SparkMAXHelper esc_tele1_helper_{
      esc_tele1_named_,
      esc_tele1,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_tele1_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_tele1_gains_helper_,
  };

  transitionLib846::Named esc_tele2_named_{*this, "esc_tele2"};

  rev::CANSparkMax esc_tele2{
      ports::telescope::kTele2_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  transitionLib846::motor::GainsHelper* esc_tele2_gains_helper_ =
      new transitionLib846::motor::GainsHelper{
          esc_tele2_named_,
          {
              0.3, /* p */
              0,   /* i */
              0.7, /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };

  transitionLib846::motor::SparkMAXHelper esc_tele2_helper_{
      esc_tele2_named_,
      esc_tele2,
      new transitionLib846::motor::SparkMAXConfigHelper{
          esc_tele2_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              15_A, /* current_limit */
          },
      },
      esc_tele2_gains_helper_,
  };

  // TODO: change gearing
  transitionLib846::Converter<units::inch_t> converter_{
      transitionLib846::kSparkMAXPeriod, transitionLib846::kSparkMAXSensorTicks,
      1_in * 2.2146};

  TelescopeReadings GetNewReadings() override;
  void WriteToHardware(TelescopeTarget target) override;
};

using OptionalTelescopeSubsystem =
    transitionLib846::OptionalSubsystem<TelescopeSubsystem, TelescopeReadings,
                                        TelescopeTarget>;

#endif  //  Y2023_SUBSYSTEMS_TELESOPE_H_