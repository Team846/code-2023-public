#include "Offseason2910Clone/subsystems/pivot.h"

#include "transitionLib846/commons.h"
#include "units/math.h"

PivotSubsystem::PivotSubsystem()
    : transitionLib846::Subsystem<PivotReadings, PivotTarget>{"pivot"} {
  esc_l1_helper_.OnInit([&] {
    esc_l1.SetInverted(true);  // CHECK THIS
    esc_l1_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_l1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_l1.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                        converter_.RealToNativePosition(0_deg));
  });

  esc_l2_helper_.OnInit([&] {
    esc_l2.SetInverted(true);  // CHECK THIS
    esc_l2_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_l2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_l2.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                        converter_.RealToNativePosition(0_deg));
  });

  esc_r1_helper_.OnInit([&] {
    esc_r1.SetInverted(false);  // CHECK THIS
    esc_r1_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_r1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_r1.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                        converter_.RealToNativePosition(0_deg));
  });

  esc_r2_helper_.OnInit([&] {
    esc_r2.SetInverted(false);  // CHECK THIS
    esc_r2_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_r2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_r2.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                        converter_.RealToNativePosition(0_deg));
  });

  esc_l1_helper_.Setup();
  esc_l2_helper_.Setup();
  esc_r1_helper_.Setup();
  esc_r2_helper_.Setup();

  has_zeroed = false;
}

void PivotSubsystem::ZeroPosition(bool zeroed) {
  if (zeroed) {
    has_zeroed = true;
    esc_l1_helper_.encoder().SetPosition(0.0);
    esc_l2_helper_.encoder().SetPosition(0.0);
    esc_r1_helper_.encoder().SetPosition(0.0);
    esc_r2_helper_.encoder().SetPosition(0.0);
  } else
    has_zeroed = false;
}

PivotTarget PivotSubsystem::ZeroTarget() const {
  PivotTarget target;
  target.output = 0.0;

  return target;
}

PivotTarget PivotSubsystem::UpTarget() const {
  PivotTarget target;
  target.output = up_speed_.value();

  return target;
}

PivotTarget PivotSubsystem::DownTarget() const {
  PivotTarget target;
  target.output = down_speed_.value();

  return target;
}

PivotTarget PivotSubsystem::ToPosition(
    transitionLib846::Pref<units::degree_t> pos) {
  PivotTarget target;
  target.output = pos.value();

  return target;
}

PivotTarget PivotSubsystem::PlacePosition() {
  PivotTarget target;

  if (readings().position < 80_deg) {
    target.output = readings().position - place_amount_.value();
  } else if (readings().position > 100_deg) {
    target.output = readings().position + place_amount_.value();
  }

  return target;
}

void PivotSubsystem::MakeAllCoastMode() {
  esc_l1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  esc_l2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  esc_r1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  esc_r2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void PivotSubsystem::MakeAllBrakeMode() {
  esc_l1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  esc_l2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  esc_r1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  esc_r2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

PivotTarget PivotSubsystem::HoldPosition() {
  PivotTarget target;
  target.output = readings().position;

  return target;
}

bool PivotSubsystem::VerifyHardware() {
  bool ok = true;
  transitionLib846_VERIFY(esc_r1_helper_.VerifyConnected(), ok,
                          "esc r1 not connected");
  transitionLib846_VERIFY(esc_r2_helper_.VerifyConnected(), ok,
                          "esc r2 not connected");
  transitionLib846_VERIFY(esc_l1_helper_.VerifyConnected(), ok,
                          "esc l1 not connected");
  transitionLib846_VERIFY(esc_l2_helper_.VerifyConnected(), ok,
                          "esc l2 not connected");

  return ok;
}

PivotReadings PivotSubsystem::GetNewReadings() {
  PivotReadings readings;

  auto pivot_position =
      converter_.NativeToRealPosition(esc_l1_helper_.encoder().GetPosition());

  position_graph_.Graph(pivot_position);

  readings.position = pivot_position;

  return readings;
}

double PivotSubsystem::custom_pid(double err, double min, double max) {
  double duty_cycle = 0.0;
  double tele_ext = transitionLib846::Commons::GetVal("tele_extension");

  duty_cycle = err * p_gain_.value() +
               ff_multiplier_.value() * (tele_ext + 16) / 48 *
                   units::math::cos(readings().position).to<double>();

  duty_cycle = std::max(min, duty_cycle);
  duty_cycle = std::min(max, duty_cycle);
  return duty_cycle;
}

void PivotSubsystem::WriteToHardware(PivotTarget target) {
  if (auto* position = std::get_if<PivotPosition>(&target.output)) {
    // Position control
    auto output = *position;

    target_position_graph_.Graph(output);

    units::degree_t position_difference = output - readings().position;

    double duty_output = 0.0;

    if (!has_zeroed) {
      esc_l1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, 0.0});
      esc_l2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, 0.0});
      esc_r1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, 0.0});
      esc_r2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, 0.0});
    } else if (units::math::abs(position_difference) < 0.9_deg) {
      duty_output = custom_pid(0, -0.07, 0.07);
      esc_l1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
      esc_l2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
      esc_r1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
      esc_r2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
    } else {
      duty_output = custom_pid(position_difference.to<double>(), -0.5, 0.5);

      if (position_difference < 0_deg && readings().position < 10_deg) {
        duty_output = custom_pid(position_difference.to<double>(), -0.1, 0.4);
      }

      if (units::math::abs(position_difference) < 12.0_deg) {
        duty_output = custom_pid(position_difference.to<double>(), -0.09, 0.09);
      }

      esc_l1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
      esc_l2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
      esc_r1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
      esc_r2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, duty_output});
    }
  } else if (auto* output = std::get_if<DutyCycle>(&target.output)) {
    if (has_zeroed) {
      // Duty Cycle control
      double speed = *output;

      esc_l1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, speed});
      esc_l2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, speed});
      esc_r1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, speed});
      esc_r2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, speed});
    }
  }
}