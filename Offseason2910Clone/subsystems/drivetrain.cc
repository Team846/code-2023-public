#include "Offseason2910Clone/subsystems/drivetrain.h"

#include <stdexcept>

#include "Offseason2910Clone/subsystems/swerve_module.h"
#include "transitionLib846/wpilib/time.h"


units::feet_per_second_t vel_readings_composite;
double vel_readings_composite_x;
double vel_readings_composite_y;

DrivetrainSubsystem::DrivetrainSubsystem()
    : transitionLib846::Subsystem<DrivetrainReadings, DrivetrainTarget>{
          "drivetrain"} {
  bearing_offset_ = 0_deg;
  ZeroOdometry();
  frc::SmartDashboard::PutData("Field", &m_field);
}

void DrivetrainSubsystem::ZeroModules() {
  Debug("Zeroed modules");
  for (auto module : modules_all_) {
    module->ZeroWithCANcoder();
  }
}

void DrivetrainSubsystem::ZeroBearing() {
  // Attempt to zero using the gyro, and retry if the gyro is disconnected or
  // calibrating

  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;
  bearing_offset_ = 0_deg;
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Debug("Gyro zero attempt {}/{}", attempts, kMaxAttempts);
    if (gyro_.IsConnected() && !gyro_.IsCalibrating()) {
      gyro_.ZeroYaw();
      Debug("Zeroed bearing");
      break;
    }

    Debug("Unable to zero", attempts, kMaxAttempts);

    if (attempts == kMaxAttempts) {
      Error("NOT ZEROED!!!");
    } else {
      Debug("Sleeping {}ms...", kSleepTimeMs);
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    }
  }
}

void DrivetrainSubsystem::ZeroOdometry() {
  bearing_offset_ = 0_deg;
  odometry_.Zero();
}

void DrivetrainSubsystem::SetPoint(
    transitionLib846::Vector2D<units::foot_t> point) {
  Debug("set point x {} y {}", point.x, point.y);
  odometry_.SetPoint(point);
}

void DrivetrainSubsystem::SetBearing(units::degree_t bearing) {
  ZeroBearing();
  bearing_offset_ = bearing;
  Debug("Zeroed bearing {}", bearing);
}

std::array<transitionLib846::Vector2D<units::feet_per_second_t>,
           DrivetrainSubsystem::kModuleCount>
DrivetrainSubsystem::SwerveControl(
    transitionLib846::Vector2D<units::feet_per_second_t> translation,
    units::degrees_per_second_t rotation_speed, units::inch_t width,
    units::inch_t height, units::inch_t radius,
    units::feet_per_second_t max_speed) {
  // Locations of each module
  static constexpr transitionLib846::Vector2D<units::dimensionless_t>
      kModuleLocationSigns[DrivetrainSubsystem::kModuleCount] = {
          {-1, +1},  // fl
          {+1, +1},  // fr
          {-1, -1},  // bl
          {+1, -1},  // br
      };

  std::array<transitionLib846::Vector2D<units::feet_per_second_t>,
             DrivetrainSubsystem::kModuleCount>
      module_targets;

  units::feet_per_second_t max_magnitude;
  for (int i = 0; i < 4; ++i) {
    // Location of the module relaive to the center
    transitionLib846::Vector2D<units::inch_t> location{
        kModuleLocationSigns[i].x * width / 2,
        kModuleLocationSigns[i].y * height / 2,
    };

    // Target direction for the module - angle from center of robot to
    // module + 90 degrees
    //
    // x and y inputs in atan2 are swapped to return a bearing
    units::degree_t direction =
        units::math::atan2(location.x, location.y) + 90_deg;

    // do 90 - direction to convert bearing to cartesian angle
    transitionLib846::Vector2D<units::feet_per_second_t> rotation{
        rotation_speed * units::math::cos(90_deg - direction) * radius / 1_rad,
        rotation_speed * units::math::sin(90_deg - direction) * radius / 1_rad,
    };

    module_targets[i] = translation + rotation;
    max_magnitude =
        units::math::max(max_magnitude, module_targets[i].Magnitude());
  }

  // Cap module speed if any of them exceed the max speed.
  // TODO unit test this.
  if (max_magnitude > max_speed) {
    auto scale = max_speed / max_magnitude;
    for (auto& t : module_targets) {
      t.x *= scale;
      t.y *= scale;
    }
  }

  return module_targets;
}

DrivetrainTarget DrivetrainSubsystem::ZeroTarget() const {
  DrivetrainTarget target;
  target.v_x = 0_fps;
  target.v_y = 0_fps;
  target.translation_reference = DrivetrainTranslationReference::kRobot;
  target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
  target.control = kOpenLoop;

  return target;
}

DrivetrainTarget DrivetrainSubsystem::LockModulesTarget() const {
  DrivetrainTarget target;
  target.v_x = 0_fps;
  target.v_y = 0_fps;
  target.translation_reference = DrivetrainTranslationReference::kRobot;
  target.rotation = DrivetrainRotationVelocity(1_deg_per_s);
  target.control = kOpenLoop;

  return target;
}

bool DrivetrainSubsystem::VerifyHardware() {
  bool ok = true;
  transitionLib846_VERIFY(gyro_.IsConnected(), ok, "gyro is not connected");
  transitionLib846_VERIFY(!gyro_.IsCalibrating(), ok, "gyro is calibrating");

  for (auto module : modules_all_) {
    bool module_ok = module->VerifyHardware();
    if (!module_ok) {
      module->Error("Failed hardware verification!!");
    }
    ok = ok && module_ok;
  }

  return ok;
}

DrivetrainReadings DrivetrainSubsystem::GetNewReadings() {
  DrivetrainReadings readings;

  readings.is_gyro_connected = gyro_.IsConnected();

  readings.pose.bearing = units::degree_t(gyro_.GetYaw()) + bearing_offset_;
  readings.angular_velocity = units::degrees_per_second_t(gyro_.GetRate());

  auto pitch_initial = units::degree_t(gyro_.GetPitch());
  auto roll_initial = units::degree_t(gyro_.GetRoll());
  auto tilt =
      0.8_deg -
      units::degree_t{pitch_initial * units::math::cos(readings.pose.bearing) +
                      roll_initial * units::math::sin(readings.pose.bearing)};

  auto current_time_ = transitionLib846::wpilib::CurrentFPGATime();

  readings.tilt = tilt;

  auto dt = current_time_ - prev_time_;
  readings.tilt_omega = (tilt - prev_tilt_) / dt;

  tilt_graph_.Graph(tilt);
  tilt_omega_graph_.Graph(readings.tilt_omega);
  prev_tilt_ = tilt;
  prev_time_ = current_time_;

  // Gets the position difference vector for each module, to update odometry
  // with
  std::array<transitionLib846::Vector2D<units::foot_t>, kModuleCount>
      module_outs;
  for (int i = 0; i < kModuleCount; ++i) {
    modules_all_[i]->UpdateReadings();
    auto d = modules_all_[i]->readings().distance;
    units::radian_t a = modules_all_[i]->readings().direction;

    transitionLib846::Vector2D<units::foot_t> vec{
        d * units::math::sin(a),
        d * units::math::cos(a),
    };
    module_outs[i] = vec;
  }

  odometry_.Update(module_outs, readings.pose.bearing);

  units::feet_per_second_t total_x = 0_fps, total_y = 0_fps;
  for (auto module : modules_all_) {
    total_x += module->readings().speed *
               units::math::cos(90_deg - module->readings().direction);
    total_y += module->readings().speed *
               units::math::sin(90_deg - module->readings().direction);
  }

  transitionLib846::Vector2D<units::feet_per_second_t> unfiltered_velocity = {
      total_x / kModuleCount, total_y / kModuleCount};

  readings.velocity = unfiltered_velocity;

  pose_x_graph_.Graph(odometry_.pose().point.x);
  pose_y_graph_.Graph(odometry_.pose().point.y);
  pose_bearing_graph.Graph(odometry_.pose().bearing);
  v_x_graph_.Graph(readings.velocity.x);
  v_y_graph_.Graph(readings.velocity.y);

  readings.pose.point = odometry_.pose().point;

  m_field.SetRobotPose(frc::Pose2d(odometry_.pose().point.y,
                                   -odometry_.pose().point.x,
                                   odometry_.pose().bearing));

  vel_readings_composite_x = units::unit_cast<double>(readings.velocity.x);

  if (vel_readings_composite_x < 0.0) {
    vel_readings_composite_x = -vel_readings_composite_x;
  }

  vel_readings_composite_y = units::unit_cast<double>(readings.velocity.y);

  if (vel_readings_composite_y < 0.0) {
    vel_readings_composite_y = -vel_readings_composite_y;
  }

  vel_readings_composite = units::feet_per_second_t(
      sqrt(vel_readings_composite_x * vel_readings_composite_x +
           vel_readings_composite_y * vel_readings_composite_y));

  return readings;
}

void DrivetrainSubsystem::WriteToHardware(DrivetrainTarget target) {
  // Graph target
  target_v_x_graph_.Graph(target.v_x);
  target_v_y_graph_.Graph(target.v_y);
  target_translation_reference_graph_.Graph(
      target.translation_reference == DrivetrainTranslationReference::kField
          ? "field"
          : "robot");
  if (auto* target_rotation =
          std::get_if<DrivetrainRotationPosition>(&target.rotation)) {
    target_rotation_position_graph_.Graph(readings().pose.bearing -
                                          *target_rotation);
    target_rotation_velocity_graph_.Graph(0_deg_per_s);
  } else if (auto* target_rotation =
                 std::get_if<DrivetrainRotationVelocity>(&target.rotation)) {
    target_rotation_position_graph_.Graph(0_deg);
    target_rotation_velocity_graph_.Graph(*target_rotation);

  } else {
    throw std::runtime_error{"unhandled case"};
  }

  transitionLib846::Vector2D<units::feet_per_second_t> target_translation = {
      target.v_x, target.v_y};
  // rotate translation vector if field oriented
  if (target.translation_reference == DrivetrainTranslationReference::kField) {
    units::degree_t offset = readings().pose.bearing;
    target_translation = target_translation.Rotate(-offset);
  }

  target_translation.x = vx_ramp_rate_.Calculate(target_translation.x);
  target_translation.y = vy_ramp_rate_.Calculate(target_translation.y);

  units::degrees_per_second_t target_omega;
  if (auto* theta = std::get_if<DrivetrainRotationPosition>(&target.rotation)) {
    // position control
    auto p_error =
        transitionLib846::CoterminalDifference(*theta, readings().pose.bearing);
    auto d_error = readings().angular_velocity;

    target_omega = units::degrees_per_second_t(
        bearing_gains_p_.value() * p_error.to<double>() +
        bearing_gains_d_.value() * d_error.to<double>());
  } else if (auto* omega =
                 std::get_if<DrivetrainRotationVelocity>(&target.rotation)) {
    // velocity control
    target_omega = *omega;
    // if (*omega != units::degrees_per_second_t(0)) {
    //   last_bearing_ = readings().pose.bearing;
    // } else {
    //   auto p_error =
    //     transitionLib846::CoterminalDifference(last_bearing_,
    //     readings().pose.bearing);
    //   auto d_error = readings().angular_velocity;

    //   target_omega = units::degrees_per_second_t(
    //       bearing_gains_p_.value() * p_error.to<double>() +
    //       bearing_gains_d_.value() * d_error.to<double>());
    // }
  } else {
    throw std::runtime_error{"unhandled case"};
  }

  // Max speed in open loop vs closed loop
  auto max_speed = target.control == kOpenLoop ? max_speed_.value()
                                               : auto_max_speed_.value();

  auto targets = SwerveControl(target_translation, target_omega, width_.value(),
                               height_.value(), module_radius_, max_speed);

  // Debug("Vel Error {}",
  // units::feet_per_second_t(sqrt(units::unit_cast<double>(target.v_x)*units::unit_cast<double>(target.v_x)+units::unit_cast<double>(target.v_y)*units::unit_cast<double>(target.v_y)))-vel_readings_composite);
  frc::SmartDashboard::PutNumber(
      "velocity_errors", (sqrt(units::unit_cast<double>(target.v_x) *
                                   units::unit_cast<double>(target.v_x) +
                               units::unit_cast<double>(target.v_y) *
                                   units::unit_cast<double>(target.v_y))) -
                             units::unit_cast<double>(vel_readings_composite));

  for (int i = 0; i < kModuleCount; ++i) {
    modules_all_[i]->SetTarget(
        {targets[i].Magnitude(), targets[i].Bearing(), target.control});
    modules_all_[i]->UpdateHardware();
  }
}