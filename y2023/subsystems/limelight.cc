#include "y2023/subsystems/limelight.h"

#include <vector>

#include "y2023/field.h"

LimelightSubsystem::LimelightSubsystem()
    : Subsystem<LimelightReadings, LimelightTarget>("limelight") {}

LimelightReadings LimelightSubsystem::GetNewReadings() {
  LimelightReadings readings;


  std::vector<double> llpython = {0.0, 0.0, 0.0};
  llpython = table_->GetEntry("llpython").GetDoubleArray(llpython);  // tv, disty, tx

  if (llpython.size() < 3) {
    Error("Wrong llpython length from limelight! len: {}", llpython.size());
    readings.target_exists = false;
  } else {
    readings.target_exists = llpython[0] != 0.0;
    readings.retro_distance = units::inch_t(llpython[1]);
    readings.tx = units::degree_t(llpython[2]);
  }

  retro_target_exists_graph_.Graph(readings.target_exists);
  retro_strafe_distance_graph_.Graph(readings.retro_distance);
  tx_graph_.Graph(readings.tx);

  return readings;
}

bool LimelightSubsystem::VerifyHardware() {
  bool ok = true;
  return ok;
}

void LimelightSubsystem::WriteToHardware(LimelightTarget target) {
  table_->PutNumber("snapshot", target.take_snapshot ? 1 : 0);
}

LimelightTarget LimelightSubsystem::ZeroTarget() const { return {}; }