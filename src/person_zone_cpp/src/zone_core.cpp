#include "person_zone_cpp/zone_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace person_zone_cpp
{
namespace
{

constexpr std::array<double, 17> kKeypointWeights{
  0.05,
  0.05, 0.05,
  0.05, 0.05,
  0.15, 0.15,
  0.08, 0.08,
  0.05, 0.05,
  0.15, 0.15,
  0.07, 0.07,
  0.05, 0.05
};

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), values.begin() + middle);
    result = (*lower + result) / 2.0;
  }
  return result;
}

}  // namespace

ZoneTracker::ZoneTracker(ZoneConfig config)
: config_(std::move(config))
{
  if (config_.confirm_frames < 1) {
    throw std::invalid_argument("confirm_frames must be at least 1");
  }
}

Observation ZoneTracker::observe(
  const std::vector<Keypoint> & keypoints,
  int image_width,
  int image_height) const
{
  Observation output;
  if (image_width <= 0 || image_height <= 0) {
    return output;
  }

  std::vector<Point2D> visible;
  std::vector<double> torso_x;
  std::vector<double> torso_y;
  double total_weight = 0.0;
  const double boundary_1 = static_cast<double>(image_width) / 3.0;
  const double boundary_2 = static_cast<double>(image_width) * 2.0 / 3.0;
  const std::size_t count = std::min(keypoints.size(), kKeypointWeights.size());

  for (std::size_t index = 0; index < count; ++index) {
    const auto & keypoint = keypoints[index];
    if (keypoint.confidence < config_.keypoint_confidence ||
      !std::isfinite(keypoint.x) || !std::isfinite(keypoint.y))
    {
      continue;
    }

    visible.push_back({keypoint.x, keypoint.y});
    if (index == 5 || index == 6 || index == 11 || index == 12) {
      torso_x.push_back(keypoint.x);
      torso_y.push_back(keypoint.y);
    }

    const double weight = kKeypointWeights[index];
    total_weight += weight;
    if (keypoint.x < boundary_1) {
      output.ratios[0] += weight;
    } else if (keypoint.x < boundary_2) {
      output.ratios[1] += weight;
    } else {
      output.ratios[2] += weight;
    }
  }

  output.visible_keypoints = visible.size();
  if (total_weight > 0.0) {
    for (auto & ratio : output.ratios) {
      ratio /= total_weight;
    }
  }

  if (torso_x.size() >= 2) {
    output.representative = Point2D{median(torso_x), median(torso_y)};
  } else if (!visible.empty()) {
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(visible.size());
    y.reserve(visible.size());
    for (const auto & point : visible) {
      x.push_back(point.x);
      y.push_back(point.y);
    }
    output.representative = Point2D{median(x), median(y)};
  }
  return output;
}

double ZoneTracker::ratio_for(
  const std::array<double, 3> & ratios,
  const std::string & state)
{
  if (state == "LEFT") {
    return ratios[0];
  }
  if (state == "CENTER") {
    return ratios[1];
  }
  if (state == "RIGHT") {
    return ratios[2];
  }
  return 0.0;
}

const std::string & ZoneTracker::update(const std::array<double, 3> & ratios)
{
  if (state_ != "NONE" && ratio_for(ratios, state_) >= config_.keep_ratio) {
    pending_.clear();
    pending_count_ = 0;
    return state_;
  }

  const auto best = std::max_element(ratios.begin(), ratios.end());
  const auto index = static_cast<std::size_t>(std::distance(ratios.begin(), best));
  static const std::array<std::string, 3> names{"LEFT", "CENTER", "RIGHT"};
  const std::string & candidate = names[index];
  const double current_ratio = ratio_for(ratios, state_);

  if (*best < config_.enter_ratio ||
    (state_ != "NONE" && candidate != state_ &&
    *best - current_ratio < config_.transition_margin))
  {
    pending_.clear();
    pending_count_ = 0;
    return state_;
  }

  if (pending_ == candidate) {
    ++pending_count_;
  } else {
    pending_ = candidate;
    pending_count_ = 1;
  }

  if (pending_count_ >= config_.confirm_frames) {
    state_ = candidate;
    pending_.clear();
    pending_count_ = 0;
  }
  return state_;
}

void ZoneTracker::reset()
{
  state_ = "NONE";
  pending_.clear();
  pending_count_ = 0;
}

const std::string & ZoneTracker::state() const
{
  return state_;
}

}  // namespace person_zone_cpp
