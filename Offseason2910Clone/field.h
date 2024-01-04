#ifndef Offseason2910Clone_FIELD_H_
#define Offseason2910Clone_FIELD_H_

#include "transitionLib846/field_point.h"
#include "transitionLib846/math.h"


// Field has blue alliance far right corner as origin
struct field {
  // autonomous routines

  struct points {
    // TESTING
    static transitionLib846::field::FieldPoint testing_point;
    static transitionLib846::Position kTestingPoint(bool should_flip) {
      return testing_point.flip(should_flip);
    };

    // SCORING
    static transitionLib846::field::FieldPoint start_scoring_table;
    static transitionLib846::Position kStartScoringTable(bool should_flip) {
      return start_scoring_table.flip(should_flip);
    };

    static transitionLib846::field::FieldPoint scoring_intermediary_point;
    static transitionLib846::Position kIntermediaryPoint(bool should_flip) {
      return scoring_intermediary_point.flip(should_flip);
    };

    static transitionLib846::field::FieldPoint scoring_intake_cube_one;
    static transitionLib846::Position kIntakeCubeOne(bool should_flip) {
      return scoring_intake_cube_one.flip(should_flip);
    };

    static transitionLib846::field::FieldPoint scoring_intake_cube_two;
    static transitionLib846::Position kIntakeCubeTwo(bool should_flip) {
      return scoring_intake_cube_two.flip(should_flip);
    };

    static transitionLib846::field::FieldPoint scoring_intake_cube_one_plus;
    static transitionLib846::Position kIntakeCubeOnePlus(bool should_flip) {
      return scoring_intake_cube_one_plus.flip(should_flip);
    };

    static transitionLib846::field::FieldPoint scoring_intake_cube_two_plus;
    static transitionLib846::Position kIntakeCubeTwoPlus(bool should_flip) {
      return scoring_intake_cube_two_plus.flip(should_flip);
    };

    static transitionLib846::field::FieldPoint scoring_score_cube;
    static transitionLib846::Position kScoreCube(bool should_flip) {
      return scoring_score_cube.flip(should_flip);
    };

    // Balance Auto
    static transitionLib846::field::FieldPoint balance_start;
    static transitionLib846::Position kStartBalance(bool should_flip) {
      return balance_start.flip(should_flip);
    };
  };

  struct length {
    static constexpr units::inch_t kFieldLength = 651.25_in;
    static constexpr units::inch_t kFieldWidth = 315.5_in;
  };
};

#endif  // Offseason2910Clone_FIELD_H_