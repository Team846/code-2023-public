#ifndef Y2023_FIELD_H_
#define Y2023_FIELD_H_

#include "frc846/math.h"

// Field has blue alliance far right corner as origin
struct field {
  // autonomous routines
  struct points {

    //TESTING
    static frc846::Position kTestingPoint(bool should_flip) {
      return flip({{-20.655_in, 350_in}, 180_deg}, should_flip);
    };

    // SCORING
    static frc846::Position kStartScoringTable(bool should_flip) {
      return flip({{-20.655_in, 82.25_in}, 180_deg}, should_flip);
    };
    static frc846::Position kScoreCone1(bool should_flip) {
      return flip({{-23.655_in, 73.75_in}, 180_deg}, should_flip);
    };
    static frc846::Position kScoreCube1(bool should_flip) {
      return flip({{-46.00_in, 73.75_in}, 180_deg}, should_flip);
    };
    static frc846::Position kIntermediaryScoring(bool should_flip) {
      return flip({{-29.695_in, 100_in}, 180_deg}, should_flip);
    };
    // Closest to scoring table
    static frc846::Position kIntake1(bool should_flip) {
      return flip({{-38.19_in, 230_in}, 180_deg}, should_flip);
    };

    // ALLIANCE WALL
    static frc846::Position kStartAllianceWall(bool should_flip) {
      return flip({{-195.345_in, 82.25_in}, 180_deg}, should_flip);
    };
    static frc846::Position kScoreCone4(bool should_flip) {
      return flip({{-192.5_in, 73.75_in}, 180_deg}, should_flip);
    };
    static frc846::Position kScoreCube4(bool should_flip) {
      return flip({{-167.53_in, 78.75_in}, 180_deg}, should_flip);
    };
    static frc846::Position kIntermediaryAlliance(bool should_flip) {
      return flip({{-187.445_in, 100_in}, 180_deg}, should_flip);
    };
    // Closest to alliance wall
    static frc846::Position kIntake4(bool should_flip) {
      return flip({{-176.19_in, 243_in}, 180_deg}, should_flip);
    };

    // MID
    static frc846::Position kStartMid(bool should_flip) {
      return flip({{-126.5_in, 73.75_in}, 180_deg}, should_flip);
    };
    static frc846::Position kMidRaise(bool should_flip) {
      return flip({{-126.5_in, 82_in}, 180_deg}, should_flip);
    };
    static frc846::Position kIntake2(bool should_flip) {
      return flip({{-84.19_in, 248_in}, 180_deg}, should_flip);
    }

    // Balancing
    // Position in front of charging station
    static frc846::Position kBalanceIntermediaryScoring(bool should_flip) {
      return flip({{-75_in, 95_in}, 180_deg}, should_flip);
    };
    static frc846::Position kBalanceForward(bool should_flip) {
      return flip({{-90.85_in, 195.5_in}, 0_deg}, should_flip);
    };

    static frc846::Position kBalanceIntermediaryAlliance(bool should_flip) {
      return flip({{-135_in, 95_in}, 180_deg}, should_flip);
    };
    static frc846::Position kBalanceBackwardAlliance(bool should_flip) {
      return flip({{-135_in, 207.5_in}, 180_deg}, should_flip);
    };

    // Only for mid + pickup piece and end auto
    static frc846::Position kBalanceBackward(bool should_flip) {
      return flip({{-90.85_in, 195.5_in}, 180_deg}, should_flip);
    };
    static frc846::Position kBalanceEnd(bool should_flip) {
      return flip({{-90.80_in, 195.5_in}, 180_deg}, should_flip);
    };
  };

  struct length {
    static constexpr units::inch_t kFieldLength = 651.25_in;
    static constexpr units::inch_t kFieldWidth = 315.5_in;
  };

  static frc846::Position flip(frc846::Position position, bool should_flip) {
    if (should_flip) {
      // Only flips coordinate, bearing flip is handled in drive command
      return {{-position.point.x, position.point.y}, -position.bearing};
    } else {
      return position;
    }
  }

  static frc846::Vector2D<units::foot_t> flip(
      frc846::Vector2D<units::foot_t> point, bool should_flip) {
    if (should_flip) {
      // Only flips coordinate, bearing flip is handled in drive command
      return {-point.x, point.y};
    } else {
      return point;
    }
  }

  static units::degree_t flip(units::degree_t bearing, bool should_flip) {
    if (should_flip) {
      // Only flips coordinate, bearing flip is handled in drive command
      return -bearing;
    } else {
      return bearing;
    }
  }
};

#endif  // Y2023_FIELD_H_