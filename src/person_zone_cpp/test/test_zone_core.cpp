#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "person_zone_cpp/zone_core.hpp"

using person_zone_cpp::Keypoint;
using person_zone_cpp::ZoneConfig;
using person_zone_cpp::ZoneTracker;

TEST(ZoneCore, WeightsAndNormalizesVisibleKeypoints)
{
  ZoneTracker tracker;
  std::vector<Keypoint> points(17, {50.0, 50.0, 1.0});
  points[5].x = 150.0;
  points[6].x = 150.0;

  const auto observation = tracker.observe(points, 300, 200);
  EXPECT_EQ(observation.visible_keypoints, 17U);
  EXPECT_NEAR(
    observation.ratios[0] + observation.ratios[1] + observation.ratios[2],
    1.0, 1e-9);
  EXPECT_GT(observation.ratios[0], observation.ratios[1]);
}

TEST(ZoneCore, ConfirmsTransitionsAndKeepsCurrentZone)
{
  ZoneConfig config;
  config.confirm_frames = 2;
  ZoneTracker tracker(config);

  EXPECT_EQ(tracker.update({0.8, 0.1, 0.1}), "NONE");
  EXPECT_EQ(tracker.update({0.8, 0.1, 0.1}), "LEFT");
  EXPECT_EQ(tracker.update({0.46, 0.44, 0.10}), "LEFT");
}

TEST(ZoneCore, RejectsWeakTransitionAndResets)
{
  ZoneConfig config;
  config.confirm_frames = 1;
  ZoneTracker tracker(config);

  EXPECT_EQ(tracker.update({0.8, 0.1, 0.1}), "LEFT");
  EXPECT_EQ(tracker.update({0.35, 0.50, 0.15}), "LEFT");
  tracker.reset();
  EXPECT_EQ(tracker.state(), "NONE");
}
