#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace person_zone_cpp
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Keypoint
{
  double x{0.0};
  double y{0.0};
  double confidence{0.0};
};

struct ZoneConfig
{
  double keypoint_confidence{0.25};
  double enter_ratio{0.60};
  double keep_ratio{0.45};
  double transition_margin{0.15};
  int confirm_frames{4};
};

struct Observation
{
  std::array<double, 3> ratios{0.0, 0.0, 0.0};
  std::optional<Point2D> representative;
  std::size_t visible_keypoints{0};
};

// ROS-free three-zone classifier. Integrators can unit-test and reuse this
// class without rclcpp, message generation, a camera driver, or a ROS distro.
class ZoneTracker
{
public:
  explicit ZoneTracker(ZoneConfig config = {});

  Observation observe(
    const std::vector<Keypoint> & keypoints,
    int image_width,
    int image_height) const;

  const std::string & update(const std::array<double, 3> & ratios);
  void reset();
  const std::string & state() const;

private:
  static double ratio_for(
    const std::array<double, 3> & ratios,
    const std::string & state);

  ZoneConfig config_;
  std::string state_{"NONE"};
  std::string pending_;
  int pending_count_{0};
};

}  // namespace person_zone_cpp
