#ifndef FRC846_CURRENT_SENSOR_H_
#define FRC846_CURRENT_SENSOR_H_

#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <deque>

#include "frc846/pref.h"
#include "units/current.h"

namespace frc846 {
class CurrentSensor {
 public:
  CurrentSensor(frc846::Pref<units::ampere_t>& setpoint);

  void Reset();
  void Calculate(units::ampere_t reading);

  units::ampere_t GetAvgCurrent();
  bool GetSpiked();
  void SetSpikeSetpoint(units::ampere_t setpoint);

 private:
  std::deque<units::ampere_t> cache;
  frc846::Pref<units::ampere_t>& spike_setpoint_;
  units::ampere_t  current_state_;
  units::ampere_t  prev_state_;
};
}  // namespace frc846

#endif  // FRC846_CURRENT_SENSOR_H_