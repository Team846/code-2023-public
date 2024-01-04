#include "y2023/subsystems/telescope.h"

TelescopeSubsystem::TelescopeSubsystem()
    : frc846::Subsystem<TelescopeReadings, TelescopeTarget>{"telescope"} {
  esc_helper_.OnInit([&] {
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    retract_limit_ = esc_.GetReverseLimitSwitch(
        rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
    extend_limit_ = esc_.GetForwardLimitSwitch(
        rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
  });
  has_zeroed_ = false;
}

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  TelescopeTarget target;
  target.output = 0.0;
  return target;
}

void TelescopeSubsystem::ResetPosition() {
  esc_helper_.encoder().SetPosition(
      converter_.RealToNativePosition(soft_limit_.value()));
  Debug("Zeroed telescope position");
}

void TelescopeSubsystem::SetHasZeroed(bool has_zeroed) {
  has_zeroed_ = has_zeroed;
}

bool TelescopeSubsystem::GetHasZeroed() { return has_zeroed_; }

TelescopeTarget TelescopeSubsystem::HoldPosition() {
  TelescopeTarget target;
  target.output = readings().position;
  return target;
}

bool TelescopeSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  // FRC846_VERIFY(retract_limit_.IsLimitSwitchEnabled(), ok,
  //               "telescoping limit switch not configured");
  // FRC846_VERIFY(extend_limit_.IsLimitSwitchEnabled(), ok,
  //               "telescoping limit switch not configured");
  FRC846_VERIFY(esc_.GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake, ok,
                "telescoping brake mode not enabled!!!!");
  return ok;
}

TelescopeReadings TelescopeSubsystem::GetNewReadings() {
  TelescopeReadings readings;

  readings.extend_limit = extend_limit_.Get();
  readings.retract_limit = retract_limit_.Get();

  auto position =
      converter_.NativeToRealPosition(esc_helper_.encoder().GetPosition());

  //ALSO SET STALL CURRENT ON THE ESC

  if (esc_.GetOutputCurrent()>spike_detection_thresh_.value() && has_zeroed_ == false) {
    has_zeroed_ = true;
    esc_helper_.encoder().SetPosition(
        converter_.RealToNativePosition(soft_limit_.value()));
    Debug("Zeroed telescope position");
  }


  has_zeroed_graph_.Graph(has_zeroed_);

  readings.position = position;
  position_graph_.Graph(position);

  return readings;
}

void TelescopeSubsystem::WriteToHardware(TelescopeTarget target) {
  extend_limit_graph_.Graph(readings().extend_limit);
  retract_limit_graph_.Graph(readings().retract_limit);

  if (!has_zeroed_) {
    esc_helper_.Write({frc846::motor::ControlMode::Percent, -0.1});
  } else {
    if (auto* position = std::get_if<Position>(&target.output)) {
      // Position

      units::turn_t output = *position;

      target_position_graph_.Graph(output);

      units::turn_t position_difference = output - readings().position;

      //The following has been disabled because the hall effect sensor has been disconnected

      // if (readings().retract_limit || readings().position < 0_tr) {
      //   if (position_difference < 0_tr) {
      //     // Can't retract further
      //     Debug("Retraction Limit!!! Can't go retract further");
      //     position_difference = 0_tr;
      //   }
      // } else if (readings().extend_limit) {
      //   if (position_difference > 0_tr) {
      //     // Can't extend further
      //     Debug("Extension limit! Can't extend further");
      //     position_difference = 0_tr;
      //   }
      // }

      esc_helper_.Write({frc846::motor::ControlMode::Position,
                         converter_.RealToNativePosition(output)});

    } else if (auto* duty_cycle = std::get_if<DutyCycle>(&target.output)) {
      // Duty Cycle
      auto output = *duty_cycle;

      //The following has been disabled because the hall effect sensor has been disconnected


      // if (readings().retract_limit || readings().position < 0_tr) {
      //   if (output < 0.0) {
      //     // Can't retract further
      //     Debug("Retraction Limit!!! Can't go retract further");
      //     output = 0.0;
      //   }
      // } else if (readings().extend_limit && output > 0.0) {
      //   // Can't extend further
      //   Debug("Extension limit! Can't extend further");
      //   output = 0.0;
      // }
      esc_helper_.Write({frc846::motor::ControlMode::Percent, output});
    }
  }
}