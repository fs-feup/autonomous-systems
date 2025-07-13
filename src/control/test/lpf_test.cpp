#include "filters/lpf.hpp"

#include "gtest/gtest.h"

/**
 * @brief Test LowPassFilter - filter()
 * Test the LowPassFilter with alpha = 1.0 (no filtering).
 */
TEST(LowPassFilterTest, No_filtering_aplied) {
  LowPassFilter lpf(1.0, 0.0);
  EXPECT_DOUBLE_EQ(lpf.filter(0.1), 0.1);
  EXPECT_DOUBLE_EQ(lpf.filter(-0.2), -0.2);
  EXPECT_DOUBLE_EQ(lpf.filter(0.35), 0.35);
}

/**
 * @brief Test LowPassFilter - filter()
 * Test the LowPassFilter with alpha = 0.0 (full filtering).
 */
TEST(LowPassFilterTest, Full_filtering_aplied) {
  LowPassFilter lpf(0.0, 0.15);
  EXPECT_DOUBLE_EQ(lpf.filter(0.3), 0.15);
  EXPECT_DOUBLE_EQ(lpf.filter(-0.3), 0.15);
}

/**
 * @brief Test LowPassFilter - filter()
 * Test the LowPassFilter with alpha = 0.5 (half filtering).
 */
TEST(LowPassFilterTest, Half_filtering_aplied) {
  LowPassFilter lpf(0.5, 0.0);
  EXPECT_DOUBLE_EQ(lpf.filter(0.2), 0.1);     // (0.5 * 0.2 + 0.5 * 0.0)
  EXPECT_DOUBLE_EQ(lpf.filter(0.4), 0.25);    // (0.5 * 0.4 + 0.5 * 0.1)
  EXPECT_DOUBLE_EQ(lpf.filter(-0.2), 0.025);  // (0.5 * -0.2 + 0.5 * 0.25)
  EXPECT_DOUBLE_EQ(lpf.filter(0.3), 0.1625);  // (0.5 * 0.3 + 0.5 * 0.025)
}

/**
 * @brief Test LowPassFilter - filter()
 * Test the LowPassFilter with alpha = 0.25 (quarter filtering).
 */
TEST(LowPassFilterTest, Quarter_filtering_aplied) {
  LowPassFilter lpf(0.25, 0.0);
  EXPECT_DOUBLE_EQ(lpf.filter(0.2), 0.05);
  EXPECT_DOUBLE_EQ(lpf.filter(0.4), 0.1375);
  EXPECT_DOUBLE_EQ(lpf.filter(-0.2), 0.053125);   // corrected value
  EXPECT_DOUBLE_EQ(lpf.filter(0.3), 0.11484375);  // corrected value
}