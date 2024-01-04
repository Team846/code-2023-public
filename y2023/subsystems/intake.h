#ifndef Y2023_SUBSYSTEMS_INTAKE_H_
#define Y2023_SUBSYSTEMS_INTAKE_H_

#include <rev/CANSparkMax.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/velocity.h>

#include "frc846/conversions.h"
#include "frc846/current_sensor.h"
#include "frc846/grapher.h"
#include "frc846/motor/helper.h"
#include "frc846/named.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "units/math.h"
#include "y2023/ports.h"

enum IntakeState {
  kStow,    // Stow
  kDeploy,  // Deploy
  kIdle,    // Do nothing
};

struct IntakeReadings {
  units::degree_t deploy_position;
  units::revolutions_per_minute_t roller_speed;
  bool stow_limit;
};
struct IntakeTarget {
  IntakeState state;
  double roller_speed;
};

class IntakeSubsystem : public frc846::Subsystem<IntakeReadings, IntakeTarget> {
 public:
  IntakeSubsystem();

  frc846::Named deploy_positions_named_{*this, "deploy_positions"};
  frc846::Pref<units::degree_t> stow_position_{deploy_positions_named_,
                                               "stow_position", 0_deg};
  frc846::Pref<units::degree_t> deploy_position_{deploy_positions_named_,
                                                 "deploy_psoition", 190_deg};

  frc846::Named roller_speeds_named_{*this, "roller_speeds"};
  frc846::Pref<double> cube_roller_speed_{roller_speeds_named_,
                                          "cube_roller_speed", 0.5};
  frc846::Pref<double> cone_roller_speed_{roller_speeds_named_,
                                          "cone_roller_speed", 0.65};

  frc846::Pref<double> reverse_roller_speed_{*this, "reverse_roller_speed",
                                             -0.8};
  frc846::Pref<double> zero_roller_speed_{*this, "zero_roller_speed", 0.0};
  frc846::Pref<bool> should_auto_stow_{*this, "auto_stow", true};

  IntakeTarget ZeroTarget() const override;
  bool VerifyHardware() override;
  void SetHasHomed(bool homed);
  void ResetPosition();

 private:
  int loops_since = 0;
  frc846::Grapher<units::degree_t> deploy_position_graph_{*this,
                                                          "deploy_position"};
  frc846::Grapher<units::degree_t> target_deploy_position_graph_{
      *this, "target_deploy_position"};
  frc846::Grapher<double> roller_speed_graph_{*this, "roller_speed"};

  frc846::Grapher<bool> has_homed_graph_{*this, "has_homed"};

  frc846::Pref<units::degree_t> stow_pause_position_{
      *this, "stow_pause_position_", 130_deg};

  frc846::Pref<units::second_t> ramp_time_{*this, "ramp_time", 0.3_s};
  frc846::Pref<double> max_output_{*this, "max_output", 0.8};
  frc846::Pref<double> max_output_reverse_{*this, "max_output_reverse", -0.4};

  bool has_homed_ = false;
  rev::CANSparkMax roller_esc_{ports::intake::kRoller_CANID,
                               rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  frc846::Named roller_esc_named_{*this, "roller_esc"};

  frc846::motor::GainsHelper* roller_esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          roller_esc_named_,
          {
              0,            /* p */
              0,            /* i */
              0,            /* d */
              0.0001836491, /* f */
              0,            /* max_integral_accumulator */
          },
      };

  frc846::motor::SparkMAXHelper roller_esc_helper_{
      roller_esc_named_, roller_esc_,
      new frc846::motor::SparkMAXConfigHelper{
          roller_esc_named_,
          {
              1.0,   // Peak output
              12_V,  // Voltage Compensation
              40_A,  // Current limit
          },
      },
      roller_esc_gains_helper_};

  rev::CANSparkMax deploy_esc_{ports::intake::kDeploy_CANID,
                               rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  frc846::Named deploy_esc_named_{*this, "deploy_esc"};

  frc846::motor::GainsHelper* deploy_esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          deploy_esc_named_,
          {
              0.55, /* p */
              0,   /* i */
              0,   /* d */
              0,   /* f */
              0,   /* max_integral_accumulator */
          },
      };
  frc846::motor::SparkMAXHelper deploy_esc_helper_{
      deploy_esc_named_, deploy_esc_,
      new frc846::motor::SparkMAXConfigHelper{
          deploy_esc_named_,
          {
              1.0,   // Peak output
              10_V,  // Voltage Compensation
              20_A,  // Current limit
          },
      },
      deploy_esc_gains_helper_};

  frc846::Converter<units::degree_t> deploy_converter_{
      frc846::kSparkMAXPeriod, frc846::kSparkMAXSensorTicks,
      1_tr / ((84 / 29) * (76 / 21) * (54 / 16) * (36 / 12))};

  static int constexpr kFreeSpeed = 5676;

  rev::SparkMaxLimitSwitch stow_limit_{deploy_esc_.GetReverseLimitSwitch(
      rev::SparkMaxLimitSwitch::Type::kNormallyClosed)};

  IntakeReadings GetNewReadings() override;
  bool has_stowed_ = true;
  void WriteToHardware(IntakeTarget target) override;
};

using OptionalIntakeSubsystem =
    frc846::OptionalSubsystem<IntakeSubsystem, IntakeReadings, IntakeTarget>;

#endif  // Y2023_SUBSYSTEMS_INTAKE_H_