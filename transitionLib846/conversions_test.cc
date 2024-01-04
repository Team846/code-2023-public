#include "transitionLib846/conversions.h"

#include <gtest/gtest.h>
#include <units/constants.h>

class ConversionsTest : public testing::Test {
 protected:
  static constexpr double kGearRatio = 1.0 / 2.0;
  static constexpr units::inch_t kWheelRadius = 2_in;
  static constexpr units::inch_t kWheelCircumference =
      2 * units::constants::pi * kWheelRadius;

  // Rotational converter (e.g. flywheel)
  //
  // native: 2048 motor ticks, 100 ms motor period
  // real:   14:50 gear ratio, 1 s period
  transitionLib846::Converter<units::degree_t> rotational_converter_{
      transitionLib846::kTalonPeriod,
      transitionLib846::kTalonFXSensorTicks,
      kGearRatio * 1_tr,
  };

  // Linear converter (e.g. drivetrain)
  //
  // native: 2048 motor ticks, 100 ms motor period
  // real:   14:50 gear ratio + 4 in wheel, 1 s period
  transitionLib846::Converter<units::foot_t> linear_converter{
      transitionLib846::kTalonPeriod,
      transitionLib846::kTalonFXSensorTicks,
      kGearRatio* kWheelCircumference,
  };
};

// Test native to real position conversion on rotational and linear converters.
TEST_F(ConversionsTest, ConverterRealPosition) {
  // native: 0 ticks
  // real:   0 turns
  EXPECT_EQ(rotational_converter_.NativeToRealPosition(0), 0_tr);
  // native: 1 tick
  // real:   1 turn / (motor resolution) * (gear ratio)
  EXPECT_EQ(rotational_converter_.NativeToRealPosition(1),
            1_tr / transitionLib846::kTalonFXSensorTicks * kGearRatio);
  // native: (motor resolution) ticks
  // real:   1 turn * (gear ratio)
  EXPECT_EQ(
      rotational_converter_.NativeToRealPosition(transitionLib846::kTalonFXSensorTicks),
      1_tr * kGearRatio);

  // native: 0 ticks
  // real:   0 in
  EXPECT_EQ(linear_converter.NativeToRealPosition(0), 0_in);
  // native: 1 tick
  // real:   1 / (motor resolution) * (gear ratio) * (wheel circumference)
  EXPECT_EQ(
      linear_converter.NativeToRealPosition(1),
      1.0 / transitionLib846::kTalonFXSensorTicks * kGearRatio * kWheelCircumference);
  // native: (motor resolution) ticks
  // real:   (wheel circumference) * (gear ratio)
  EXPECT_EQ(linear_converter.NativeToRealPosition(transitionLib846::kTalonFXSensorTicks),
            kWheelCircumference * kGearRatio);
}

// Test native to real velocity conversion on rotational and linear converters.
TEST_F(ConversionsTest, ConverterRealVelocity) {
  // native: 0 ticks / 100 ms
  // real:   0 turns / 1s
  EXPECT_EQ(rotational_converter_.NativeToRealVelocity(0), 0_tr / 1_s);
  // native: 1 tick / 100 ms
  // real:   1 turn / (motor resolution) * (gear ratio) * (1 s / 100 ms) / 1 s
  EXPECT_EQ(
      rotational_converter_.NativeToRealVelocity(1),
      1_tr / transitionLib846::kTalonFXSensorTicks * kGearRatio * (1_s / 100_ms) / 1_s);
  // native: (motor resolution) ticks / 100 ms
  // real:   1 turn * (gear ratio) * (1 s / 100 ms) / 1 s
  EXPECT_EQ(
      rotational_converter_.NativeToRealVelocity(transitionLib846::kTalonFXSensorTicks),
      1_tr * kGearRatio * (1_s / 100_ms) / 1_s);

  // native: 0 ticks / 100 ms
  // real:   0 in / 1 s
  EXPECT_EQ(linear_converter.NativeToRealVelocity(0), 0_in / 1_s);
  // native: 1 tick / 100 ms
  // real:   1 / (motor resolution) * (gear ratio) * (wheel circumference) * (
  //         1 s / 100 ms) / 1 s
  EXPECT_EQ(linear_converter.NativeToRealVelocity(1),
            1.0 / transitionLib846::kTalonFXSensorTicks * kGearRatio *
                kWheelCircumference * (1_s / 100_ms) / 1_s);
  // native: (motor resolution) ticks / 100 ms
  // real:   (wheel circumference) * (gear ratio) * (1 s / 100 ms) / 1 s
  EXPECT_EQ(linear_converter.NativeToRealVelocity(transitionLib846::kTalonFXSensorTicks),
            kWheelCircumference * kGearRatio * (1_s / 100_ms) / 1_s);
}

// Test real to native position conversion on rotational and linear converters.
TEST_F(ConversionsTest, ConverterNativePosition) {
  // real:   0 turns
  // native: 0 ticks
  EXPECT_EQ(rotational_converter_.RealToNativePosition(0_tr), 0);
  // real:   1 turn
  // native: 1 / (gear ratio) * (motor resolution) ticks
  EXPECT_EQ(rotational_converter_.RealToNativePosition(1_tr),
            1 / kGearRatio * transitionLib846::kTalonFXSensorTicks);

  // real:   0 in
  // native: 0 ticks
  EXPECT_EQ(linear_converter.RealToNativePosition(0_in), 0);
  // real:   (wheel circumference) in
  // native: 1 / (gear ratio) * (motor resolution) ticks
  EXPECT_EQ(linear_converter.RealToNativePosition(kWheelCircumference),
            1 / kGearRatio * transitionLib846::kTalonFXSensorTicks);
  // real:   1 in
  // native: 1 / (wheel circumference) / (gear ratio) * (motor resolution) ticks
  EXPECT_EQ(linear_converter.RealToNativePosition(1_in),
            1 / kWheelCircumference.value() / kGearRatio *
                transitionLib846::kTalonFXSensorTicks);
}

// Test real to native velocity conversion on rotational and linear converters.
TEST_F(ConversionsTest, ConverterNativeVelocity) {
  // real:   0 turns / 1s
  // native: 0 ticks / 100 ms
  EXPECT_EQ(rotational_converter_.RealToNativeVelocity(0_tr / 1_s), 0);
  // real:   1 turn / 1 s
  // native: 1 / (gear ratio) * (motor resolution) * (100 ms / 1s)
  EXPECT_EQ(rotational_converter_.RealToNativeVelocity(1_tr / 1_s),
            1 / kGearRatio * transitionLib846::kTalonFXSensorTicks * 100_ms / 1_s);

  // real:   0 in / 1 s
  // native: 0 ticks / 100ms
  EXPECT_EQ(linear_converter.RealToNativeVelocity(0_in / 1_s), 0);
  // real:   (wheel circumference) in / 1 s
  // native: 1 / (gear ratio) * (motor resolution) ticks * (100 ms / 1s)
  EXPECT_EQ(linear_converter.RealToNativeVelocity(kWheelCircumference / 1_s),
            1 / kGearRatio * transitionLib846::kTalonFXSensorTicks * 100_ms / 1_s);
  // real:   1 in / 1s
  // native: 1 / (wheel circumference) / (gear ratio) * (motor resolution) ticks
  //         * (100 ms / 1s)
  EXPECT_EQ(linear_converter.RealToNativeVelocity(1_in / 1_s),
            1 / kWheelCircumference.to<double>() / kGearRatio *
                transitionLib846::kTalonFXSensorTicks * 100_ms / 1_s);
}
