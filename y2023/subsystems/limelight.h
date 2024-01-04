#ifndef Y2023_SUBSYSTEMS_LIMELIGHT_H_
#define Y2023_SUBSYSTEMS_LIMELIGHT_H_

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "frc846/grapher.h"
#include "frc846/named.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"

struct LimelightReadings {
  bool target_exists;  // same as tv

  units::inch_t retro_distance;
  units::degree_t tx;
};

struct LimelightTarget {
  bool take_snapshot;
};

class LimelightSubsystem
    : public frc846::Subsystem<LimelightReadings, LimelightTarget> {
 public:
  units::inch_t camera_offset = 9.125_in;
  LimelightSubsystem();

  LimelightTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Grapher<units::inch_t> retro_strafe_distance_graph_{
      *this, "retro_strafe_distance"};

  frc846::Grapher<bool> retro_target_exists_graph_{*this, "target_exists"};

  frc846::Grapher<units::angle::degree_t> tx_graph_{*this, "tx_graph"};

  std::shared_ptr<nt::NetworkTable> table_ =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  LimelightReadings GetNewReadings() override;

  void WriteToHardware(LimelightTarget target) override;
};

using OptionalLimelightSubsystem =
    frc846::OptionalSubsystem<LimelightSubsystem, LimelightReadings,
                              LimelightTarget>;

#endif  // Y2023_SUBSYSTEMS_LIMELIGHT_H_