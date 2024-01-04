#include "field.h"

#include "transitionLib846/field_point.h"

transitionLib846::field::FieldPoint field::points::testing_point{
    "testing_point", {23.665_in, 193.75_in}, 0_deg};

transitionLib846::field::FieldPoint field::points::start_scoring_table{
    "scoring_start", {23.655_in, 73.75_in}, 0_deg};
transitionLib846::field::FieldPoint field::points::scoring_intermediary_point{
    "scoring_intermediary_point", {23.655_in, 210.75_in}, 0_deg};
transitionLib846::field::FieldPoint field::points::scoring_intake_cube_one{
    "scoring_intake_cube_one", {28.655_in, 260.75_in}, 0_deg};
transitionLib846::field::FieldPoint field::points::scoring_intake_cube_two{
    "scoring_intake_cube_two", {73.655_in, 256.75_in}, 0_deg};
transitionLib846::field::FieldPoint field::points::scoring_intake_cube_one_plus{
    "scoring_intake_cube_one_plus", {28.655_in, 275.75_in}, 0_deg};
transitionLib846::field::FieldPoint field::points::scoring_intake_cube_two_plus{
    "scoring_intake_cube_two_plus", {73.655_in, 279.75_in}, 0_deg};
transitionLib846::field::FieldPoint field::points::scoring_score_cube{
    "scoring_score_cube", {48.655_in, 73.75_in}, 0_deg};

transitionLib846::field::FieldPoint field::points::balance_start{
    "balance_start", {90.655_in, 73.75_in}, 0_deg};