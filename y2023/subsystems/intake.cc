#include "y2023/subsystems/intake.h"

#include <variant>

#include "frc846/current_sensor.h"
#include "frc846/motor/helper.h"
#include "units/constants.h"

IntakeSubsystem::IntakeSubsystem()
    : frc846::Subsystem<IntakeReadings, IntakeTarget>("intake") {
  roller_esc_helper_.OnInit([this] {
    roller_esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
  });
  deploy_esc_helper_.OnInit([this] {
    deploy_esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    deploy_esc_helper_.DisableStatusFrames({
        rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
    });
    deploy_esc_helper_.pid_controller().SetOutputRange(-max_output_.value(),
                                                       max_output_.value());
    deploy_esc_.SetClosedLoopRampRate(ramp_time_.value().value());
  });

  roller_esc_helper_.Setup();
  deploy_esc_helper_.Setup();
}

IntakeTarget IntakeSubsystem::ZeroTarget() const {
  IntakeTarget target;
  target.state = IntakeState::kIdle;
  target.roller_speed = 0.0;
  return target;
}

bool IntakeSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(roller_esc_helper_.VerifyConnected(), ok,
                "roller esc not connected");
  FRC846_VERIFY(deploy_esc_helper_.VerifyConnected(), ok,
                "deploy esc not connected");
  return ok;
}

void IntakeSubsystem::SetHasHomed(bool homed) { has_homed_ = homed; }
void IntakeSubsystem::ResetPosition() {
  has_homed_ = true;
  deploy_esc_helper_.encoder().SetPosition(0.0);
}

IntakeReadings IntakeSubsystem::GetNewReadings() {
  IntakeReadings readings;
  readings.deploy_position = deploy_converter_.NativeToRealPosition(
      deploy_esc_helper_.encoder().GetPosition());

  deploy_position_graph_.Graph(readings.deploy_position);

  readings.stow_limit = stow_limit_.Get();
  if (readings.stow_limit) {
    deploy_esc_helper_.encoder().SetPosition(0.0);
    has_homed_ = true;
  }
  has_homed_graph_.Graph(has_homed_);

  return readings;
}

void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  double roller_speed = target.roller_speed;

  units::degree_t current_position = readings().deploy_position;
  units::degree_t deploy_position = current_position;
  // Deploy motor

  if (target.state == kDeploy) {
    deploy_position = deploy_position_.value();
    loops_since = 0;
    has_stowed_ = false;
  } else if (target.state == kStow) {
    // STOW
    loops_since++;
    if (loops_since < 75 && !has_stowed_) {
      deploy_position = stow_pause_position_.value();
    } else {
      has_stowed_ = true;
      deploy_position = stow_position_.value();
    }

    if (units::math::abs(current_position - stow_position_.value()) > 10_deg) {
      roller_speed = cube_roller_speed_.value();
    }
  } else if (target.state == kIdle) {
    loops_since = 0;
    deploy_position = current_position;
  }

  deploy_esc_helper_.Write(
      {frc846::motor::ControlMode::Position,
       deploy_converter_.RealToNativePosition(deploy_position)});

  target_deploy_position_graph_.Graph(deploy_position);
  roller_speed_graph_.Graph(roller_esc_helper_.encoder().GetVelocity() -
                            kFreeSpeed * roller_speed);
  roller_esc_helper_.Write(
      {frc846::motor::ControlMode::Velocity, kFreeSpeed * roller_speed});
}