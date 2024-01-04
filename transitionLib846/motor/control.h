#ifndef transitionLib846_ESC_CONTROL_H_
#define transitionLib846_ESC_CONTROL_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <rev/CANSparkMax.h>

#include <initializer_list>

#include "transitionLib846/ctre_namespace.h"
#include "transitionLib846/motor/gains.h"
#include "transitionLib846/loggable.h"
#include <units/current.h>

transitionLib846_CTRE_NAMESPACE()

namespace transitionLib846::motor {

static constexpr units::millisecond_t kCANTimeout = 50_ms;

enum ControlMode { Percent, Velocity, Position, Current };

constexpr ctre::ControlMode CTREControlMode(ControlMode mode) {
  switch (mode) {
    case ControlMode::Percent:
      return ctre::ControlMode::PercentOutput;
    case ControlMode::Velocity:
      return ctre::ControlMode::Velocity;
    case ControlMode::Position:
      return ctre::ControlMode::Position;
    case ControlMode::Current:
      return ctre::ControlMode::Current;
    default:
      throw std::runtime_error("Unsupported control type");
  }
}

class SparkMaxController {
    public:
    SparkMaxController(Loggable parent_, std::string name, int kCanIdPort, 
        rev::CANSparkMaxLowLevel::MotorType motor_type = rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    ~SparkMaxController();

    int Setup(units::millisecond_t timeout, GainsHelper* gainsHelper,
        bool isInverted = false, rev::CANSparkMax::IdleMode kIdleMode = rev::CANSparkMax::IdleMode::kCoast);

    int Setup(GainsHelper* gainsHelper, bool isInverted = false,
        rev::CANSparkMax::IdleMode kIdleMode = rev::CANSparkMax::IdleMode::kCoast);

    void ZeroEncoder(double pos);

    private:
    Loggable obj;
    GainsHelper* gains_helper;

    rev::CANSparkMax esc_;
    Gains gains_cache_;

    rev::SparkMaxPIDController pid_controller_;
    rev::SparkMaxRelativeEncoder encoder_;
};

}

#endif