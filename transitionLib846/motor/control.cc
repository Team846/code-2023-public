#include "control.h"
#include "transitionLib846/motor/gains.h"

namespace transitionLib846::motor {
    SparkMaxController::SparkMaxController(Loggable parent_, std::string name, int kCanIdPort, 
        rev::CANSparkMaxLowLevel::MotorType motor_type) : obj(parent_, name), 
            esc_{kCanIdPort, motor_type}, pid_controller_{esc_.GetPIDController()},
                encoder_{esc_.GetEncoder()} {}
    
    SparkMaxController::~SparkMaxController() {
        if (gains_helper != nullptr) {
            delete gains_helper;
        }
    }

    int SparkMaxController::Setup(units::millisecond_t timeout, GainsHelper* gainsHelper, bool isInverted, rev::CANSparkMax::IdleMode kIdleMode) {
        esc_.SetCANTimeout(timeout.to<double>());
        esc_.SetIdleMode(kIdleMode);
        esc_.GetEncoder().SetInverted(isInverted);
        gains_helper = gainsHelper;

        if (gains_helper != nullptr) {
            gains_helper->Write(pid_controller_, gains_cache_, true);
        }

        esc_.SetSmartCurrentLimit(gainsHelper->current_limit_.value());
        esc_.EnableVoltageCompensation(12.0);
        
        return 0;
    }

    int SparkMaxController::Setup(GainsHelper* gainsHelper, bool isInverted, rev::CANSparkMax::IdleMode kIdleMode) {
        return SparkMaxController::Setup(kCANTimeout, gainsHelper, isInverted, kIdleMode);
    }

    //use converter, be able to set to position
    void SparkMaxController::ZeroEncoder(double pos) {
        encoder_.SetPosition(0.0);
    }
}