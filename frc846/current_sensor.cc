#include "frc846/current_sensor.h"

#define CACHE_SIZE 20

namespace frc846 {
CurrentSensor::CurrentSensor(frc846::Pref<units::ampere_t>& setpoint)
    : spike_setpoint_(setpoint) {
  Reset();
}

bool CurrentSensor::GetSpiked() {
  return current_state_ > spike_setpoint_.value();
}

units::ampere_t CurrentSensor::GetAvgCurrent() { return current_state_; }

void CurrentSensor::Reset() {
  cache.clear();
  for (int i = 0; i < CACHE_SIZE; i++) {
    cache.push_back(units::ampere_t(0));
  }
  prev_state_ = units::ampere_t(0);
  current_state_ = units::ampere_t(0);
}

void CurrentSensor::Calculate(units::ampere_t reading) {
  cache.pop_front();
  cache.push_back(reading);

  // Calculate average current over the cache size, or circular buffer window
  units::ampere_t sum = units::ampere_t(0);
  for (int i = 0; i < CACHE_SIZE; i++) {
    sum += cache.at(i);
  }

  current_state_ = sum / CACHE_SIZE;
  prev_state_ = current_state_;
}
}  // namespace frc846