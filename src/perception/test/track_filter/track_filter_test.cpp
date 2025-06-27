#include "track_filtering/track_filter.hpp"

#include <gtest/gtest.h>

TEST(TrackFilterTest, KeepsConesWithEnoughNeighbors) {
  std::vector<PreCone> cones = {
      PreCone(0.0, 0.0, false, 1.0), PreCone(1.0, 0.0, false, 1.0), PreCone(0.0, 1.0, false, 1.0),
      PreCone(5.0, 5.0, false, 1.0)  // far away cone, should be removed
  };

  TrackFilter filter(0.1, 2.0, 2);  // Need at least 2 cones within 2.0 units
  filter.filter(cones);

  EXPECT_EQ(cones.size(), 3);
  EXPECT_TRUE(std::find(cones.begin(), cones.end(), PreCone(0.0, 0.0, false, 1.0)) != cones.end());
  EXPECT_TRUE(std::find(cones.begin(), cones.end(), PreCone(1.0, 0.0, false, 1.0)) != cones.end());
  EXPECT_TRUE(std::find(cones.begin(), cones.end(), PreCone(0.0, 1.0, false, 1.0)) != cones.end());
}

TEST(TrackFilterTest, RemovesTooCloseCones) {
  std::vector<PreCone> cones = {PreCone(0.0, 0.0, false, 1.0),   // too close
                                PreCone(0.05, 0.0, false, 1.0),  // too close
                                PreCone(2.0, 0.0, false, 1.0)};

  TrackFilter filter(0.1, 5.0, 1);
  filter.filter(cones);

  // The 0.0 and 0.05 cones are too close, so they will be removed
  EXPECT_EQ(cones.size(), 1);
}

TEST(TrackFilterTest, RemovesIsolatedCone) {
  std::vector<PreCone> cones = {PreCone(0.0, 0.0, false, 1.0), PreCone(1.0, 1.0, false, 1.0),
                                PreCone(10.0, 10.0, false, 1.0)};

  TrackFilter filter(0.1, 2.0, 1);  // Cone at (10, 10) is isolated
  filter.filter(cones);

  EXPECT_EQ(cones.size(), 2);
  EXPECT_EQ(cones[0].get_position(), common_lib::structures::Position(0.0, 0.0));
  EXPECT_EQ(cones[1].get_position(), common_lib::structures::Position(1.0, 1.0));
}
