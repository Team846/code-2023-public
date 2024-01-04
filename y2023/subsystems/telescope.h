#ifndef Y2023_SUBSYSTEMS_TELESCOPE_H_
#define Y2023_SUBSYSTEMS_TELESCOPE_H_

#include <frc/DigitalInput.h>
#include <units/angle.h>

#include "frc846/conversions.h"
#include "frc846/grapher.h"
#include "frc846/motor/helper.h"
#include "frc846/named.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "y2023/ports.h"

enum TelescopeControlMode {
  kDutyCycle,
  kPosition,
};

struct TelescopeReadings {
  bool extend_limit;
  bool retract_limit;
  units::turn_t position;
};

using Position = units::turn_t;
using DutyCycle = double;

struct TelescopeTarget {
  std::variant<Position, DutyCycle> output;
};

class TelescopeSubsystem
    : public frc846::Subsystem<TelescopeReadings, TelescopeTarget> {
 public:
  TelescopeSubsystem();

  frc846::Named speeds_named_{*this, "telescope_speeds"};

  frc846::Pref<double> extend_speed_{speeds_named_, "extend_speed", 0.4};
  frc846::Pref<double> retract_speed_{speeds_named_, "retract_speed", -0.4};

  frc846::Pref<double> spike_detection_thresh_{speeds_named_, "spike_detection_thresh", 14.0};

  frc846::Named positions_named_{*this, "telescope_positions"};

  frc846::Pref<units::turn_t> retract_position_{positions_named_,
                                                "retract_position", 0_tr};

  frc846::Pref<units::turn_t> shelf_position_{positions_named_,
                                              "shelf_position", 0_tr};
  frc846::Pref<units::turn_t> max_retract_position_{
      positions_named_, "max_retract_position", 0.5_tr};

  frc846::Pref<units::turn_t> max_extension_{positions_named_, "max_extension",
                                             3.12_tr};

  frc846::Named cone_named_{positions_named_, "cone"};
  frc846::Pref<units::turn_t> cone_mid_position_{cone_named_,
                                                 "cone_mid_position", 0.5_tr};
  frc846::Pref<units::turn_t> cone_high_position_{
      cone_named_, "cone_high_position", 3.35_tr};
  frc846::Pref<units::turn_t> cone_conveyor_position_{cone_named_,
                                                      "cone_conveyor", 1.3_tr};

  frc846::Named cube_named_{positions_named_, "cube"};
  frc846::Pref<units::turn_t> cube_mid_position_{cube_named_,
                                                 "cube_mid_position", 0_tr};
  frc846::Pref<units::turn_t> cube_high_position_{cube_named_,
                                                  "cube_high_position", 1.7_tr};
  frc846::Pref<units::turn_t> cube_conveyor_position_{cube_named_,
                                                      "cube_conveyor", 0.3_tr};

  frc846::Pref<units::turn_t> soft_limit_{positions_named_, "soft_limit",
                                          -0.07_tr};
  frc846::Pref<units::turn_t> tolerance_{positions_named_, "tolerance", 0.1_tr};

  void Zero();

  TelescopeTarget ZeroTarget() const override;
  TelescopeTarget HoldPosition();
  void SetHasZeroed(bool has_zeroed);
  bool GetHasZeroed();
  void ResetPosition();

  bool VerifyHardware() override;

 private:
  frc846::Grapher<units::turn_t> target_position_graph_{*this,
                                                        "target_position"};
  frc846::Grapher<bool> has_zeroed_graph_{*this, "has_zeroed"};
  frc846::Converter<units::turn_t> converter_{
      frc846::kSparkMAXPeriod,
      frc846::kSparkMAXSensorTicks,
      // 2x 3:1 MAXPlanetary, 4:1
      1_tr / ((3 / 1) * (3 / 1) * (4 / 1)),
  };
  frc846::Named esc_named_{*this, "esc"};

  rev::CANSparkMax esc_{
      ports::telescope::kTelescoping_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  frc846::motor::GainsHelper* esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          esc_named_,
          {
              0.09, /* p */
              0,    /* i */
              0,    /* d */
              0,    /* f */
              0,    /* max_integral_accumulator */
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
              10_A, /* current_limit */
          },
      },
      esc_gains_helper_,
  };

  frc846::Grapher<bool> extend_limit_graph_{*this, "extend_limit"};
  frc846::Grapher<bool> retract_limit_graph_{*this, "retract_limit"};
  frc846::Grapher<units::turn_t> position_graph_{*this, "position"};

  rev::SparkMaxLimitSwitch retract_limit_{esc_.GetReverseLimitSwitch(
      rev::SparkMaxLimitSwitch::Type::kNormallyClosed)};
  rev::SparkMaxLimitSwitch extend_limit_{esc_.GetForwardLimitSwitch(
      rev::SparkMaxLimitSwitch::Type::kNormallyClosed)};

  bool has_zeroed_ = false;

  int loops_since_ = 0;

  TelescopeReadings GetNewReadings() override;
  void WriteToHardware(TelescopeTarget target) override;
};

using OptionalTelescopeSubsystem =
    frc846::OptionalSubsystem<TelescopeSubsystem, TelescopeReadings,
                              TelescopeTarget>;

#endif  //  Y2023_SUBSYSTEMS_TELESCOPE_H_