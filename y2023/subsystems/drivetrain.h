#ifndef Y2023_SUBSYSTEMS_DRIVETRAIN_H_
#define Y2023_SUBSYSTEMS_DRIVETRAIN_H_

#include <AHRS.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>
#include <variant>

#include "frc/filter/SlewRateLimiter.h"
#include "frc846/conversions.h"
#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/motor/config.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "frc846/swerve_odometry.h"
#include "frc846/wpilib/time.h"
#include "y2023/ports.h"
#include "y2023/subsystems/swerve_module.h"

struct DrivetrainReadings {
  bool is_gyro_connected;
  frc846::Position pose;
  units::degrees_per_second_t angular_velocity;
  frc846::Vector2D<units::feet_per_second_t> velocity;
  units::degree_t tilt;
  units::degrees_per_second_t tilt_omega;
};



// Robot vs field oriented translation control.
enum DrivetrainTranslationReference { kRobot, kField };

// Position control of drivetrain steering.
using DrivetrainRotationPosition = units::degree_t;

// Velocity control of drivetrain steering.
using DrivetrainRotationVelocity = units::degrees_per_second_t;

// Drivetrain rotation target.
using DrivetrainRotation =
    std::variant<DrivetrainRotationPosition, DrivetrainRotationVelocity>;

struct DrivetrainTarget {
  units::feet_per_second_t v_x;
  units::feet_per_second_t v_y;
  DrivetrainTranslationReference translation_reference;

  DrivetrainRotation rotation;
  DrivetrainControl control;
};

class DrivetrainSubsystem
    : public frc846::Subsystem<DrivetrainReadings, DrivetrainTarget> {
 public:
  DrivetrainSubsystem();

  // Number of swerve modules (avoid hardcoding 4 in loops and such).
  static constexpr int kModuleCount = 4;

  // Zero the modules with their CANCoders.
  void ZeroModules();

  // Zero bearing with the gyro.
  void ZeroBearing();

  // Zero bearing and reset odometry to zero.
  void ZeroOdometry();

  // Set odometry point.
  void SetPoint(frc846::Vector2D<units::foot_t> point);

  // Set bearing.
  void SetBearing(units::degree_t bearing);

  // Max drivetrain speed (NEO SDS Mk4i L1 -> 12 theoretical).
  frc846::Pref<units::feet_per_second_t> max_speed_{*this, "max_speed",
                                                    14.2_fps};

  frc846::Pref<units::feet_per_second_t> vx_ramp_rate_limit{
      *this, "ramp_rate_vx", 30_fps};
  frc846::Pref<units::feet_per_second_t> vy_ramp_rate_limit{
      *this, "ramp_rate_vy", 100_fps};

  frc::SlewRateLimiter<units::fps> vx_ramp_rate_{vx_ramp_rate_limit.value() /
                                                 1_s};
  frc::SlewRateLimiter<units::fps> vy_ramp_rate_{vy_ramp_rate_limit.value() /
                                                 1_s};

  // Closed loop tuned for this
  frc846::Pref<units::feet_per_second_t> auto_max_speed_{
      *this, "auto_max_speed", 11.2_fps};

  frc846::Pref<double> slow_mode_percent_{*this, "slow_mode_percent", 0.04};
  frc846::Pref<double> slow_omega_percent_{*this, "slow_omega_percent", 0.12};
  frc846::Pref<double> pov_control_speed_{*this, "pov_control_speed_", 1.0};
  frc846::Pref<double> max_horizontal_strafe_{*this, "pov_control_speed_", 10.0};

  frc846::Pref<units::feet_per_second_t> velocity_error{*this, "velocity_error", 0_fps};

  // Max turning speed.
  units::degrees_per_second_t max_omega() const {
    return max_speed_.value() / module_radius_ * 1_rad *
           percent_max_omega_.value();
  }

  // Max drivetrain acceleration for trajectory generation.
  frc846::Pref<units::feet_per_second_squared_t> max_acceleration_{
      *this, "max_acceleration", 8_fps_sq};

  // Max drivetrain deceleration for trajectory generation.
  frc846::Pref<units::feet_per_second_squared_t> max_deceleration_{
      *this, "max_deceleration", 10_fps_sq};

  // Lookahead distance during trajectory following.
  frc846::Pref<units::inch_t> extrapolation_distance_{
      *this, "extrapolation_distance", 8_in};

  frc846::Pref<units::degrees_per_second_t> angular_velocity_threshold_{
      *this, "angular_velocity_threshold", 1_deg_per_s};

  frc846::Named balance_named_{*this, "balance"};
  // Auto Balance gains
  frc846::Named balance_gains_named_{balance_named_, "balance_gains"};
  frc846::Pref<double> balance_gains_p_{balance_gains_named_, "p", 0.05};
  frc846::Pref<double> balance_gains_d_{balance_gains_named_, "d", 0};

  frc846::Named align_gains_named_{*this, "align_gains"};
  frc846::Pref<double> align_gains_p_{align_gains_named_, "p", 3.5};

  // Auto align tolerance
  frc846::Pref<units::inch_t> align_tolerance_{align_gains_named_,
                                               "align_tolerance", 0.3_in};

  // Auto Balance Thresholds
  frc846::Pref<units::degree_t> angle_threshold_{balance_named_,
                                                 "angle_threshold", 0.001_deg};
  frc846::Pref<units::feet_per_second_t> balance_max_speed_{
      balance_named_, "balance_max_speed", 2.8_fps};

  // Convert a translation vector and the drivetrain angular velocity to the
  // individual module outputs.
  static std::array<frc846::Vector2D<units::feet_per_second_t>, kModuleCount>
  SwerveControl(frc846::Vector2D<units::feet_per_second_t> translation,
                units::degrees_per_second_t rotation_speed, units::inch_t width,
                units::inch_t height, units::inch_t radius,
                units::feet_per_second_t max_speed);

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Drivetrain dimensions.
  frc846::Pref<units::inch_t> width_{*this, "width", 21.75_in};
  frc846::Pref<units::inch_t> height_{*this, "height", 26.75_in};

  // How much to scale the max turning speed by.
  frc846::Pref<double> percent_max_omega_{*this, "percent_max_omega", 0.45};

  // Distance from center of robot to module.
  units::inch_t module_radius_ =
      units::math::sqrt(units::math::pow<2>(width_.value() / 2) +
                        units::math::pow<2>(height_.value() / 2));

  // Wheel radius for odometry. 4" wheels.
  frc846::Pref<units::inch_t> wheel_radius_{*this, "wheel_radius", 1.93_in};

  // Rotation position gains.
  frc846::Named bearing_gains_named_{*this, "bearing_gains"};
  frc846::Pref<double> bearing_gains_p_{bearing_gains_named_, "p", 8.3};
  frc846::Pref<double> bearing_gains_d_{bearing_gains_named_, "d", -4.7};

  units::degree_t prev_tilt_ = 0_deg;
  units::second_t prev_time_ = 0_s;

  // Pose graphers.
  frc846::Named pose_named_{*this, "pose"};
  frc846::Grapher<units::foot_t> pose_x_graph_{pose_named_, "x"};
  frc846::Grapher<units::foot_t> pose_y_graph_{pose_named_, "y"};
  frc846::Grapher<units::degree_t> pose_bearing_graph{pose_named_, "bearing"};

  // Balance graphers.
  frc846::Grapher<units::degree_t> tilt_graph_{balance_named_, "tilt"};
  frc846::Grapher<units::degrees_per_second_t> tilt_omega_graph_{balance_named_,
                                                                 "tilt_omega"};

  // Velocity graphers.
  frc846::Named velocity_named_{*this, "velocity"};
  frc846::Grapher<units::feet_per_second_t> v_x_graph_{velocity_named_, "v_x"};
  frc846::Grapher<units::feet_per_second_t> v_y_graph_{velocity_named_, "v_y"};

  // Target graphers.
  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<units::feet_per_second_t> target_v_x_graph_{target_named_,
                                                              "v_x"};
  frc846::Grapher<units::feet_per_second_t> target_v_y_graph_{target_named_,
                                                              "v_y"};
  frc846::Grapher<std::string> target_translation_reference_graph_{
      target_named_,
      "translation_reference",
  };
  frc846::Grapher<units::degree_t> target_rotation_position_graph_{
      target_named_, "rotation_position"};
  frc846::Grapher<units::degrees_per_second_t> target_rotation_velocity_graph_{
      target_named_, "rotation_velocity"};

  frc846::SwerveOdometry odometry_;
  units::angle::degree_t bearing_offset_;

  frc846::Named drive_esc_named_{*this, "drive_esc"};
  frc846::Named steer_esc_named_{*this, "steer_esc"};
  frc::Field2d m_field;

  frc846::motor::SparkMAXConfigHelper* drive_esc_config_helper_ =
      new frc846::motor::SparkMAXConfigHelper{
          drive_esc_named_,
          {
              1.0,   // peak output
              12_V,  // voltage comp saturation
              80_A,  // NEO smart current limit
          },
      };
  frc846::motor::GainsHelper* drive_esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          drive_esc_named_,
          {
              0.0002,    /* p */
              0,         /* i */
              0,         /* d */
              0.0001769, /* f */
              0,         /* max_integral_accumulator */
          },
      };

  frc846::motor::SparkMAXConfigHelper* steer_esc_config_helper_ =
      new frc846::motor::SparkMAXConfigHelper{
          drive_esc_named_,
          {
              1.0,   // peak output
              12_V,  // voltage comp saturation
              40_A,  // peak current limit (continuous)
          },

      };
  frc846::motor::GainsHelper* steer_esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          steer_esc_named_,
          {
              0.12, /* p */
              0,    /* i */
              0,    /* d */
              0,    /* f */
              0,    /* max_integral_accumulator */
          },
      };

  frc846::Converter<units::foot_t> drive_converter_{
      frc846::kSparkMAXPeriod,
      frc846::kSparkMAXSensorTicks,
      (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) *
          frc846::Circumference(wheel_radius_.value()),
  };
  frc846::Converter<units::degree_t> steer_converter_{
      frc846::kSparkMAXPeriod,
      frc846::kSparkMAXSensorTicks,
      (7.0 / 150.0) * 1_tr,
  };

  SwerveModuleSubsystem module_fl_{
      *this,
      "FL",
      3.23_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kFLDrive_CANID,
      ports::drivetrain::kFLSteer_CANID,
      ports::drivetrain::kFLCANCoder_CANID,
  };

  SwerveModuleSubsystem module_fr_{
      *this,
      "FR",
      140.05_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kFRDrive_CANID,
      ports::drivetrain::kFRSteer_CANID,
      ports::drivetrain::kFRCANCoder_CANID,
  };

  SwerveModuleSubsystem module_bl_{
      *this,
      "BL",
      297.5_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kBLDrive_CANID,
      ports::drivetrain::kBLSteer_CANID,
      ports::drivetrain::kBLCANCoder_CANID,
  };

  SwerveModuleSubsystem module_br_{
      *this,
      "BR",
      157.89_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kBRDrive_CANID,
      ports::drivetrain::kBRSteer_CANID,
      ports::drivetrain::kBRCANCoder_CANID,
  };

  SwerveModuleSubsystem* modules_all_[kModuleCount]{&module_fl_, &module_fr_,
                                                    &module_bl_, &module_br_};

  AHRS gyro_{frc::SPI::Port::kMXP};

  DrivetrainReadings GetNewReadings() override;

  void WriteToHardware(DrivetrainTarget target) override;
};

#endif  // Y2023_SUBSYSTEMS_DRIVETRAIN_H_