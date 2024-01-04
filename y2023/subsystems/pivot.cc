#include "y2023/subsystems/pivot.h"

#include "units/math.h"

PivotSubsystem::PivotSubsystem()
    : frc846::Subsystem<PivotReadings, PivotTarget>{"pivot"} {
  esc_helper_.OnInit([&] {
    esc_.SetInverted(true);
    esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                      converter_.RealToNativePosition(0_deg));
  });

  esc_helper_.Setup();
}

void PivotSubsystem::Zero() {
  Debug("Zeroed pivot encoder");
  esc_helper_.encoder().SetPosition(
      converter_.RealToNativePosition(readings().pot_position));
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  PivotTarget target;
  target.output = 0.0;

  return target;
}

PivotTarget PivotSubsystem::HoldPosition() {
  PivotTarget target;
  target.output = readings().position;

  return target;
}

bool PivotSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  FRC846_VERIFY(esc_.GetInverted() == true, ok,
                "pivot esc incorrect invert state");
  return ok;
}

PivotReadings PivotSubsystem::GetNewReadings() {
  PivotReadings readings;

  // -0.02_V for pot lower range
  auto raw_pot = units::volt_t(potentiometer_.GetVoltage()) - kLowerPotVoltage;
  if (kInvertedPot) {
    // Pot should follow same direction as the motor encoder
    raw_pot = kPotVoltRange - raw_pot;
  }

  auto pot_position = ((raw_pot - potentiometer_zero_voltage_.value()) /
                       kPotVoltRange * kPotRange / kSprocketRatio) +
                      potentiometer_zero_pivot_position_.value();
  auto pivot_position =
      converter_.NativeToRealPosition(esc_helper_.encoder().GetPosition());

  pot_voltage_graph_.Graph(raw_pot);
  pot_position_graph_.Graph(pot_position);
  position_graph_.Graph(pivot_position);

  readings.raw_pot = raw_pot;
  readings.pot_position = pot_position;
  readings.position = pivot_position;

  return readings;
}

void PivotSubsystem::WriteToHardware(PivotTarget target) {
  if (auto* position = std::get_if<PivotPosition>(&target.output)) {
    // Position control
    auto output = *position;

    target_position_graph_.Graph(output);

    units::degree_t position_difference = output - readings().position;

    // First make sure we're still in the range of xV - yV on the pot
    if ((readings().raw_pot < kMinLimit && position_difference < 0_deg) ||
        (readings().raw_pot > kMaxLimit && position_difference > 0_deg)) {
      Warn("EXCEEDED POT LIMITS pot: {}, position: {}", readings().raw_pot,
           position_difference);
      position_difference = 0.0_deg;
    }

    if (units::math::abs(position_difference) < 0.5_deg) {
      esc_helper_.Write({frc846::motor::ControlMode::Percent, 0.0});
    } else {
      // Limit speed when retracting
      if (position_difference < 0_deg && readings().position < 10_deg) {
        esc_helper_.pid_controller().SetOutputRange(-0.4, 1.0);
      } else {
        esc_helper_.pid_controller().SetOutputRange(-1.0, 1.0);
      }

      esc_helper_.Write({frc846::motor::ControlMode::Position,
                         converter_.RealToNativePosition(output)});
    }
  } else if (auto* output = std::get_if<DutyCycle>(&target.output)) {
    // Duty Cycle control
    double speed = *output;

    // First make sure we're still in the range of xV - yV on the pot
    if ((readings().raw_pot < kMinLimit && speed < 0) ||
        (readings().raw_pot > kMaxLimit && speed > 0)) {
      Warn("EXCEEDED POT LIMITS pot: {}, pivot speed: {}", readings().raw_pot,
           speed);
      speed = 0.0;
    }
    esc_helper_.Write({frc846::motor::ControlMode::Percent, speed});
  }
}