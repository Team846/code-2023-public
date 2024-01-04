#ifndef Offseason2910Clone_SUBSYSTEMS_LIMELIGHT_H_
#define Offseason2910Clone_SUBSYSTEMS_LIMELIGHT_H_

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "transitionLib846/grapher.h"
#include "transitionLib846/named.h"
#include "transitionLib846/pref.h"
#include "transitionLib846/subsystem.h"

struct LimelightReadings {
  bool target_exists;  // same as tv

  units::inch_t retro_distance;
  units::degree_t tx;
};

struct LimelightTarget {
  bool take_snapshot;
};

class LimelightSubsystem
    : public transitionLib846::Subsystem<LimelightReadings, LimelightTarget> {
 public:
  units::inch_t camera_offset = 9.125_in;
  LimelightSubsystem();

  LimelightTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  transitionLib846::Grapher<units::inch_t> retro_strafe_distance_graph_{
      *this, "retro_strafe_distance"};

  transitionLib846::Grapher<bool> retro_target_exists_graph_{*this,
                                                             "target_exists"};

  transitionLib846::Grapher<units::angle::degree_t> tx_graph_{*this,
                                                              "tx_graph"};

  std::shared_ptr<nt::NetworkTable> table_ =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  LimelightReadings GetNewReadings() override;

  void WriteToHardware(LimelightTarget target) override;
};

using OptionalLimelightSubsystem =
    transitionLib846::OptionalSubsystem<LimelightSubsystem, LimelightReadings,
                                        LimelightTarget>;

#endif  // Offseason2910Clone_SUBSYSTEMS_LIMELIGHT_H_