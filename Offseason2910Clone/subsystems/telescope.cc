#include "Offseason2910Clone/subsystems/telescope.h"

#include "transitionLib846/commons.h"
#include "units/math.h"

TelescopeSubsystem::TelescopeSubsystem()
    : transitionLib846::Subsystem<TelescopeReadings, TelescopeTarget>{
          "telescope"} {
  esc_tele1_helper_.OnInit([&] {
    esc_tele1.SetInverted(true);
    esc_tele1_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_tele1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_tele1.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                           converter_.RealToNativePosition(0_in));
  });

  esc_tele2_helper_.OnInit([&] {
    esc_tele2.SetInverted(true);
    esc_tele2_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0});
    esc_tele2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    esc_tele2.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse,
                           converter_.RealToNativePosition(0_in));
  });

  esc_tele1_helper_.Setup();
  esc_tele2_helper_.Setup();
  has_zeroed = false;
}
void TelescopeSubsystem::SetHasZeroed(bool zeroed) {
  if (zeroed) {
    has_zeroed = true;
    esc_tele1_helper_.encoder().SetPosition(0.0);
    esc_tele2_helper_.encoder().SetPosition(0.0);
  } else {
    has_zeroed = false;
  }
}

TelescopeTarget TelescopeSubsystem::ZeroTarget() const {
  TelescopeTarget target;
  target.output = 0.0;

  return target;
}

TelescopeTarget TelescopeSubsystem::OutTarget() const {
  TelescopeTarget target;
  target.output = 0.2;

  return target;
}

TelescopeTarget TelescopeSubsystem::InTarget() const {
  TelescopeTarget target;
  target.output = -0.13;

  return target;
}

TelescopeTarget TelescopeSubsystem::ToPosition(units::inch_t pos) {
  TelescopeTarget target;
  target.output = pos;
  return target;
}

TelescopeTarget TelescopeSubsystem::ToPosition(
    transitionLib846::Pref<units::inch_t> pos) {
  TelescopeTarget target;
  target.output = pos.value();
  return target;
}

TelescopeTarget TelescopeSubsystem::HoldPosition() {
  TelescopeTarget target;
  target.output = readings().extension;

  return target;
}

bool TelescopeSubsystem::VerifyHardware() {
  bool ok = true;
  transitionLib846_VERIFY(esc_tele1_helper_.VerifyConnected(), ok,
                          "esc tele1 not connected");
  transitionLib846_VERIFY(esc_tele2_helper_.VerifyConnected(), ok,
                          "esc tele2 not connected");

  return ok;
}

TelescopeReadings TelescopeSubsystem::GetNewReadings() {
  TelescopeReadings readings;

  auto telescope_position = converter_.NativeToRealPosition(
      esc_tele1_helper_.encoder().GetPosition());

  position_graph_.Graph(telescope_position);

  readings.extension = telescope_position;

  transitionLib846::Commons::SetVal("tele_extension",
                                    readings.extension.to<double>());

  return readings;
}

void TelescopeSubsystem::MakeAllCoast() {
  esc_tele1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  esc_tele2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void TelescopeSubsystem::MakeAllBrake() {
  esc_tele1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  esc_tele2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

// teleop controls
void TelescopeSubsystem::WriteToHardware(TelescopeTarget target) {
  if (auto* position = std::get_if<TelescopePosition>(&target.output)) {
    // Position control
    auto output = *position;

    target_position_graph_.Graph(output);

    units::inch_t position_difference = output - readings().extension;
    // TODO: change speed values
    if (!has_zeroed) {
      esc_tele1_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, 0.0});
      esc_tele2_helper_.Write(
          {transitionLib846::motor::ControlMode::Percent, 0.0});

    } else {
      esc_tele1_helper_.pid_controller().SetOutputRange(-0.07, 0.4);
      if (readings().extension < 4_in && position_difference < 0_in) {
        esc_tele1_helper_.pid_controller().SetOutputRange(-0.03, 0.06);
      }

      esc_tele1_helper_.Write({transitionLib846::motor::ControlMode::Position,
                               converter_.RealToNativePosition(output)});
      esc_tele2_helper_.Write({transitionLib846::motor::ControlMode::Position,
                               converter_.RealToNativePosition(output)});
    }
  } else if (!has_zeroed) {
    esc_tele1_helper_.Write(
        {transitionLib846::motor::ControlMode::Percent, 0.0});
    esc_tele2_helper_.Write(
        {transitionLib846::motor::ControlMode::Percent, 0.0});

  } else if (auto* output = std::get_if<DutyCycle>(&target.output)) {
    double speed = *output;

    esc_tele1_helper_.Write(
        {transitionLib846::motor::ControlMode::Percent, speed});
    esc_tele2_helper_.Write(
        {transitionLib846::motor::ControlMode::Percent, speed});
  }
}