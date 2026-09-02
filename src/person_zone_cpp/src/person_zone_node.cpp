#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "person_pose_msgs/msg/person_keypoints.hpp"
#include "person_zone_cpp/zone_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class PersonZoneNode : public rclcpp::Node
{
public:
  PersonZoneNode()
  : Node("person_zone_node")
  {
    kp_conf_ = declare_parameter<double>("keypoint_confidence", 0.25);
    enter_ratio_ = declare_parameter<double>("enter_ratio", 0.60);
    keep_ratio_ = declare_parameter<double>("keep_ratio", 0.45);
    transition_margin_ =
      declare_parameter<double>("transition_margin", 0.15);
    margin_ = declare_parameter<int>("boundary_margin_px", 10);
    confirm_frames_ = declare_parameter<int>("confirm_frames", 4);
    lost_frames_ = declare_parameter<int>("lost_person_frames", 20);

    min_depth_ = declare_parameter<int>("min_depth_mm", 600);
    max_depth_ = declare_parameter<int>("max_depth_mm", 5000);
    off_x_ = declare_parameter<int>("depth_offset_x", -7);
    off_y_ = declare_parameter<int>("depth_offset_y", 7);
    patch_r_ = declare_parameter<int>("depth_patch_radius", 7);
    flip_depth_ = declare_parameter<bool>("flip_depth", false);

    exit_left_ratio_ =
      declare_parameter<double>("exit_left_ratio", 0.15);
    exit_right_ratio_ =
      declare_parameter<double>("exit_right_ratio", 0.15);
    exit_top_ratio_ =
      declare_parameter<double>("exit_top_ratio", 0.15);
    exit_bottom_ratio_ =
      declare_parameter<double>("exit_bottom_ratio", 0.15);
    enabled_exit_directions_ =
      declare_parameter<std::vector<std::string>>(
      "enabled_exit_directions", {"LEFT"});
    auto_clear_warning_ =
      declare_parameter<bool>("auto_clear_warning", true);
    warning_clear_frames_ =
      declare_parameter<int>("warning_clear_frames", 5);
    require_same_person_to_clear_ =
      declare_parameter<bool>("require_same_person_to_clear", false);
    reid_max_distance_ =
      declare_parameter<double>("reid_max_normalized_distance", 0.35);
    stale_timeout_sec_ =
      declare_parameter<double>("stale_timeout_sec", 1.0);
    event_history_path_ =
      declare_parameter<std::string>(
      "event_history_path",
      "");

    zone_tracker_ = std::make_unique<person_zone_cpp::ZoneTracker>(
      person_zone_cpp::ZoneConfig{
        kp_conf_, enter_ratio_, keep_ratio_, transition_margin_, confirm_frames_});

    const auto kp_topic =
      declare_parameter<std::string>(
      "keypoints_topic", "/person/keypoints");

    const auto depth_topic =
      declare_parameter<std::string>(
      "depth_topic", "/camera/depth/image_raw");

    const auto info_topic =
      declare_parameter<std::string>(
      "camera_info_topic", "/camera/depth/camera_info");

    const auto state_topic =
      declare_parameter<std::string>(
      "state_topic", "/person_zone_state");

    const auto pos_topic =
      declare_parameter<std::string>(
      "position_topic", "/person_position");

    const auto exit_topic =
      declare_parameter<std::string>(
      "exit_direction_topic", "/person_exit_direction");

    const auto emergency_topic =
      declare_parameter<std::string>(
      "emergency_topic", "/person_emergency");

    const auto edge_topic =
      declare_parameter<std::string>(
      "edge_warning_topic", "/person_edge_warning");

    const auto event_topic =
      declare_parameter<std::string>(
      "exit_event_topic", "/person_exit_event");

    const auto diagnostics_topic =
      declare_parameter<std::string>(
      "diagnostics_topic", "/person_zone/diagnostics");

    state_pub_ =
      create_publisher<std_msgs::msg::String>(state_topic, 10);

    pos_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>(
      pos_topic, 10);

    exit_pub_ =
      create_publisher<std_msgs::msg::String>(exit_topic, 10);

    emergency_pub_ =
      create_publisher<std_msgs::msg::Bool>(
      emergency_topic, 10);

    edge_pub_ =
      create_publisher<std_msgs::msg::String>(edge_topic, 10);

    event_pub_ =
      create_publisher<std_msgs::msg::String>(event_topic, 10);

    diagnostics_pub_ =
      create_publisher<std_msgs::msg::String>(diagnostics_topic, 10);

    warning_ack_srv_ =
      create_service<std_srvs::srv::Trigger>(
      "/person_warning_ack",
      std::bind(
        &PersonZoneNode::warningAck,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    kp_sub_ =
      create_subscription<person_pose_msgs::msg::PersonKeypoints>(
      kp_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &PersonZoneNode::kpCb,
        this,
        std::placeholders::_1));

    depth_sub_ =
      create_subscription<sensor_msgs::msg::Image>(
      depth_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &PersonZoneNode::depthCb,
        this,
        std::placeholders::_1));

    info_sub_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
        &PersonZoneNode::infoCb,
        this,
        std::placeholders::_1));

    diagnostics_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PersonZoneNode::publishDiagnostics, this));

    RCLCPP_INFO(get_logger(), "C++ zone node started");
  }

private:
  using P = person_zone_cpp::Point2D;

  enum class ExitDirection
  {
    NONE,
    LEFT,
    RIGHT,
    TOP,
    BOTTOM
  };

  struct SkeletonSignature
  {
    std::array<P, 17> points{};
    std::array<bool, 17> valid{};
  };

  struct Position3D
  {
    double x;
    double y;
    double z;
  };

  static double median(std::vector<double> values)
  {
    if (values.empty()) {
      return 0.0;
    }

    const std::size_t middle = values.size() / 2;

    std::nth_element(
      values.begin(),
      values.begin() + middle,
      values.end());

    double result = values[middle];

    if (values.size() % 2 == 0) {
      const auto it =
        std::max_element(
        values.begin(),
        values.begin() + middle);

      result = (*it + result) / 2.0;
    }

    return result;
  }

  void depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_depth_time_ = now();
    if (msg->encoding != "16UC1" &&
      msg->encoding != "mono16")
    {
      return;
    }

    dw_ = static_cast<int>(msg->width);
    dh_ = static_cast<int>(msg->height);
    step_ = static_cast<int>(msg->step);

    depth_ = msg->data;
    frame_id_ = msg->header.frame_id;
    have_depth_ = true;
  }

  void infoCb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    last_info_time_ = now();
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];

    have_info_ = fx_ > 0.0 && fy_ > 0.0;

    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }
  }

  uint16_t rawDepth(int x, int y) const
  {
    if (!have_depth_ ||
      x < 0 || y < 0 ||
      x >= dw_ || y >= dh_)
    {
      return 0;
    }

    const int source_x =
      flip_depth_ ? dw_ - 1 - x : x;

    const std::size_t offset =
      static_cast<std::size_t>(y) *
      static_cast<std::size_t>(step_) +
      static_cast<std::size_t>(source_x) * 2;

    if (offset + 1 >= depth_.size()) {
      return 0;
    }

    uint16_t value = 0;
    std::memcpy(
      &value,
      depth_.data() + offset,
      sizeof(uint16_t));

    return value;
  }

  std::optional<double> patchDepth(
    int x,
    int y) const
  {
    std::vector<double> values;

    for (int yy = y - patch_r_; yy <= y + patch_r_; ++yy) {
      for (int xx = x - patch_r_; xx <= x + patch_r_; ++xx) {
        const auto depth = rawDepth(xx, yy);

        if (depth >= min_depth_ &&
          depth <= max_depth_)
        {
          values.push_back(depth);
        }
      }
    }

    if (values.size() < 5) {
      return std::nullopt;
    }

    return median(values);
  }

  person_zone_cpp::Observation observe(
    const person_pose_msgs::msg::PersonKeypoints & msg) const
  {
    std::vector<person_zone_cpp::Keypoint> keypoints;
    const std::size_t count =
      std::min({
        msg.x.size(),
        msg.y.size(),
        msg.confidence.size()
      });
    keypoints.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      keypoints.push_back({msg.x[i], msg.y[i], msg.confidence[i]});
    }
    return zone_tracker_->observe(
      keypoints,
      static_cast<int>(msg.image_width),
      static_cast<int>(msg.image_height));
  }

  void update(
    const std::array<double, 3> & ratio)
  {
    const auto old_state = state_;
    state_ = zone_tracker_->update(ratio);
    if (old_state != state_) {
      RCLCPP_INFO(
        get_logger(),
        "State changed: %s -> %s",
        old_state.c_str(),
        state_.c_str());
    }
  }

  void publishState()
  {
    std_msgs::msg::String msg;
    msg.data = state_;
    state_pub_->publish(msg);
  }

  void publishExit()
  {
    std_msgs::msg::String msg;
    msg.data = exit_direction_;
    exit_pub_->publish(msg);
  }

  void publishEmergency()
  {
    std_msgs::msg::Bool msg;
    msg.data = emergency_;
    emergency_pub_->publish(msg);
  }

  void publishStatus()
  {
    publishState();
    publishEmergency();
  }

  static std::string exitDirectionName(
    ExitDirection direction)
  {
    switch (direction) {
      case ExitDirection::LEFT:
        return "LEFT";
      case ExitDirection::RIGHT:
        return "RIGHT";
      case ExitDirection::TOP:
        return "TOP";
      case ExitDirection::BOTTOM:
        return "BOTTOM";
      case ExitDirection::NONE:
      default:
        return "NONE";
    }
  }

  bool exitDirectionEnabled(
    ExitDirection direction) const
  {
    const auto name = exitDirectionName(direction);
    return std::find(
      enabled_exit_directions_.begin(),
      enabled_exit_directions_.end(),
      name) != enabled_exit_directions_.end();
  }

  ExitDirection classifyExit() const
  {
    if (!have_last_representative_ ||
      last_image_width_ <= 0)
    {
      return ExitDirection::NONE;
    }

    const double width =
      static_cast<double>(last_image_width_);
    const double height =
      static_cast<double>(last_image_height_);

    const std::array<ExitDirection, 4> directions{
      ExitDirection::LEFT,
      ExitDirection::RIGHT,
      ExitDirection::TOP,
      ExitDirection::BOTTOM
    };

    for (const auto direction : directions) {
      if (!exitDirectionEnabled(direction)) {
        continue;
      }
      if (direction == ExitDirection::LEFT &&
        last_representative_.x <= width * exit_left_ratio_)
      {
        return direction;
      }
      if (direction == ExitDirection::RIGHT &&
        last_representative_.x >= width * (1.0 - exit_right_ratio_))
      {
        return direction;
      }
      if (direction == ExitDirection::TOP &&
        last_representative_.y <= height * exit_top_ratio_)
      {
        return direction;
      }
      if (direction == ExitDirection::BOTTOM &&
        last_representative_.y >= height * (1.0 - exit_bottom_ratio_))
      {
        return direction;
      }
    }

    return ExitDirection::NONE;
  }

  std::string edgeDirection(
    const P & point,
    int width,
    int height) const
  {
    const double image_width = static_cast<double>(width);
    const double image_height = static_cast<double>(height);
    if (exitDirectionEnabled(ExitDirection::LEFT) &&
      point.x <= image_width * exit_left_ratio_)
    {
      return "LEFT";
    }
    if (exitDirectionEnabled(ExitDirection::RIGHT) &&
      point.x >= image_width * (1.0 - exit_right_ratio_))
    {
      return "RIGHT";
    }
    if (exitDirectionEnabled(ExitDirection::TOP) &&
      point.y <= image_height * exit_top_ratio_)
    {
      return "TOP";
    }
    if (exitDirectionEnabled(ExitDirection::BOTTOM) &&
      point.y >= image_height * (1.0 - exit_bottom_ratio_))
    {
      return "BOTTOM";
    }
    return "NONE";
  }

  SkeletonSignature makeSignature(
    const person_pose_msgs::msg::PersonKeypoints & msg) const
  {
    SkeletonSignature signature;
    const std::size_t count =
      std::min<std::size_t>(
      17,
      std::min({msg.x.size(), msg.y.size(), msg.confidence.size()}));

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (std::size_t i = 0; i < count; ++i) {
      if (msg.confidence[i] < kp_conf_ ||
        !std::isfinite(msg.x[i]) ||
        !std::isfinite(msg.y[i]))
      {
        continue;
      }
      signature.valid[i] = true;
      signature.points[i] = P{msg.x[i], msg.y[i]};
      min_x = std::min(min_x, static_cast<double>(msg.x[i]));
      min_y = std::min(min_y, static_cast<double>(msg.y[i]));
      max_x = std::max(max_x, static_cast<double>(msg.x[i]));
      max_y = std::max(max_y, static_cast<double>(msg.y[i]));
    }

    const double scale = std::max(max_x - min_x, max_y - min_y);
    if (!std::isfinite(scale) || scale < 1.0) {
      signature.valid.fill(false);
      return signature;
    }

    const double center_x = (min_x + max_x) / 2.0;
    const double center_y = (min_y + max_y) / 2.0;
    for (std::size_t i = 0; i < signature.points.size(); ++i) {
      if (signature.valid[i]) {
        signature.points[i].x =
          (signature.points[i].x - center_x) / scale;
        signature.points[i].y =
          (signature.points[i].y - center_y) / scale;
      }
    }
    return signature;
  }

  bool samePerson(
    const SkeletonSignature & current) const
  {
    if (!have_exit_signature_) {
      return true;
    }
    double distance_sum = 0.0;
    int overlap = 0;
    for (std::size_t i = 0; i < current.points.size(); ++i) {
      if (!current.valid[i] || !exit_signature_.valid[i]) {
        continue;
      }
      distance_sum += std::hypot(
        current.points[i].x - exit_signature_.points[i].x,
        current.points[i].y - exit_signature_.points[i].y);
      ++overlap;
    }
    return overlap >= 4 &&
           distance_sum / static_cast<double>(overlap) <= reid_max_distance_;
  }

  void clearWarning(const std::string & reason)
  {
    emergency_ = false;
    exit_direction_ = "NONE";
    warning_clear_n_ = 0;
    publishEmergency();
    publishExit();
    RCLCPP_INFO(
      get_logger(), "Warning cleared (%s)", reason.c_str());
  }

  void warningAck(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!emergency_) {
      response->success = true;
      response->message = "Warning is already clear";
      return;
    }
    clearWarning("manual acknowledgement");
    response->success = true;
    response->message = "Warning cleared";
  }

  void publishExitEvent(ExitDirection direction)
  {
    const auto stamp = now();
    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"timestamp\":" << stamp.seconds()
         << ",\"direction\":\"" << exitDirectionName(direction)
         << "\",\"last_zone\":\"" << last_zone_before_loss_
         << "\",\"last_x\":" << last_representative_.x
         << ",\"last_y\":" << last_representative_.y;
    if (last_position_) {
      json << ",\"x\":" << last_position_->x
           << ",\"y\":" << last_position_->y
           << ",\"z\":" << last_position_->z;
    }
    json << "}";

    std_msgs::msg::String event;
    event.data = json.str();
    event_pub_->publish(event);

    if (event_history_path_.empty()) {
      return;
    }
    try {
      const std::filesystem::path path(event_history_path_);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
      const bool write_header =
        !std::filesystem::exists(path) ||
        std::filesystem::file_size(path) == 0;
      std::ofstream output(path, std::ios::app);
      if (write_header) {
        output << "timestamp,direction,last_zone,last_x,last_y,x,y,z\n";
      }
      output << std::fixed << std::setprecision(3)
             << stamp.seconds() << ","
             << exitDirectionName(direction) << ","
             << last_zone_before_loss_ << ","
             << last_representative_.x << ","
             << last_representative_.y << ",";
      if (last_position_) {
        output << last_position_->x << ","
               << last_position_->y << ","
               << last_position_->z;
      } else {
        output << ",,";
      }
      output << "\n";
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Could not write exit history: %s", error.what());
    }
  }

  void publishDiagnostics()
  {
    const auto current = now();
    std::vector<std::string> warnings;
    if (last_kp_time_.nanoseconds() == 0 ||
      (current - last_kp_time_).seconds() > stale_timeout_sec_)
    {
      warnings.push_back("POSE_STALE");
    }
    if (last_depth_time_.nanoseconds() == 0 ||
      (current - last_depth_time_).seconds() > stale_timeout_sec_)
    {
      warnings.push_back("DEPTH_STALE");
    }
    std_msgs::msg::String msg;
    msg.data = warnings.empty() ? "OK" : "";
    for (std::size_t i = 0; i < warnings.size(); ++i) {
      if (i > 0) {
        msg.data += "|";
      }
      msg.data += warnings[i];
    }
    diagnostics_pub_->publish(msg);
  }

  void publishPos(
    const person_pose_msgs::msg::PersonKeypoints & msg,
    const P & point)
  {
    if (!have_depth_) {
      return;
    }

    int x =
      static_cast<int>(
      point.x * dw_ /
      static_cast<double>(msg.image_width)) +
      off_x_;

    int y =
      static_cast<int>(
      point.y * dh_ /
      static_cast<double>(msg.image_height)) +
      off_y_;

    x = std::clamp(x, 0, dw_ - 1);
    y = std::clamp(y, 0, dh_ - 1);

    const auto depth = patchDepth(x, y);

    if (!depth) {
      return;
    }

    const double z = *depth / 1000.0;

    const double fx =
      have_info_ ? fx_ : 580.0;

    const double fy =
      have_info_ ? fy_ : 580.0;

    const double cx =
      have_info_ ? cx_ :
      static_cast<double>(dw_) / 2.0;

    const double cy =
      have_info_ ? cy_ :
      static_cast<double>(dh_) / 2.0;

    geometry_msgs::msg::PointStamped output;

    output.header.stamp = now();
    output.header.frame_id = frame_id_;

    output.point.x =
      (static_cast<double>(x) - cx) * z / fx;

    output.point.y =
      (static_cast<double>(y) - cy) * z / fy;

    output.point.z = z;

    last_position_ = Position3D{
      output.point.x,
      output.point.y,
      output.point.z
    };
    pos_pub_->publish(output);
  }

  void noPerson()
  {
    ++lost_n_;
    warning_clear_n_ = 0;

    std_msgs::msg::String edge_msg;
    edge_msg.data = "NONE";
    edge_pub_->publish(edge_msg);

    if (tracking_person_ &&
      lost_n_ >= lost_frames_ &&
      !lost_classified_)
    {
      const auto exit = classifyExit();
      const bool warning_was_latched = emergency_;
      if (exit != ExitDirection::NONE || !warning_was_latched) {
        exit_direction_ = exitDirectionName(exit);
      }
      emergency_ =
        warning_was_latched || exit != ExitDirection::NONE;
      lost_classified_ = true;
      tracking_person_ = false;
      exit_signature_ = last_signature_;
      have_exit_signature_ = have_last_signature_;

      if (state_ != "NONE") {
        RCLCPP_INFO(
          get_logger(),
          "State changed: %s -> NONE",
          state_.c_str());
      }

      state_ = "NONE";
      zone_tracker_->reset();
      pending_.clear();
      pending_n_ = 0;

      if (exit != ExitDirection::NONE &&
        !exit_sent_)
      {
        publishExit();
        publishExitEvent(exit);
        exit_sent_ = true;
        RCLCPP_ERROR(
          get_logger(),
          "EMERGENCY: person exited through %s boundary",
          exit_direction_.c_str());
      } else {
        publishExit();
        RCLCPP_WARN(
          get_logger(),
          "Person lost without a supported exit direction");
      }
    }

    publishStatus();
  }

  void kpCb(
    const person_pose_msgs::msg::PersonKeypoints::SharedPtr msg)
  {
    last_kp_time_ = now();

    if (!msg->detected ||
      msg->image_width == 0 ||
      msg->image_height == 0)
    {
      noPerson();
      return;
    }

    const auto observation = observe(*msg);
    const auto & ratio = observation.ratios;
    const auto & representative_point = observation.representative;

    if (observation.visible_keypoints < 2 ||
      !representative_point)
    {
      noPerson();
      return;
    }

    const bool detected_again =
      !tracking_person_;
    const auto current_signature = makeSignature(*msg);

    if (detected_again && have_last_representative_) {
      RCLCPP_INFO(
        get_logger(),
        "Person detected again");
    }

    lost_n_ = 0;
    lost_classified_ = false;
    tracking_person_ = true;
    exit_sent_ = false;

    if (emergency_) {
      const bool identity_matches =
        !require_same_person_to_clear_ ||
        samePerson(current_signature);
      warning_clear_n_ =
        identity_matches ? warning_clear_n_ + 1 : 0;
      if (auto_clear_warning_ &&
        warning_clear_n_ >= warning_clear_frames_)
      {
        clearWarning("stable person re-detection");
      }
    } else {
      warning_clear_n_ = 0;
      exit_direction_ = "NONE";
      if (detected_again) {
        publishExit();
      }
    }

    last_image_width_ =
      static_cast<int>(msg->image_width);

    last_image_height_ =
      static_cast<int>(msg->image_height);

    last_representative_ = *representative_point;
    have_last_representative_ = true;
    last_signature_ = current_signature;
    have_last_signature_ = true;

    const auto edge = edgeDirection(
      *representative_point,
      static_cast<int>(msg->image_width),
      static_cast<int>(msg->image_height));
    std_msgs::msg::String edge_msg;
    edge_msg.data = edge;
    edge_pub_->publish(edge_msg);

    update(ratio);

    if (state_ != "NONE") {
      last_zone_before_loss_ = state_;
    }

    publishStatus();
    publishPos(*msg, *representative_point);
  }

  double kp_conf_;
  double enter_ratio_;
  double keep_ratio_;
  double transition_margin_;

  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;

  double exit_left_ratio_;
  double exit_right_ratio_;
  double exit_top_ratio_;
  double exit_bottom_ratio_;
  double reid_max_distance_;
  double stale_timeout_sec_;

  int margin_;
  int confirm_frames_;
  int lost_frames_;
  int warning_clear_frames_;

  int min_depth_;
  int max_depth_;
  int off_x_;
  int off_y_;
  int patch_r_;

  bool flip_depth_;
  bool have_depth_ = false;
  bool have_info_ = false;
  bool have_last_representative_ = false;
  bool have_last_signature_ = false;
  bool have_exit_signature_ = false;
  bool tracking_person_ = false;
  bool lost_classified_ = false;
  bool exit_sent_ = false;
  bool emergency_ = false;
  bool auto_clear_warning_;
  bool require_same_person_to_clear_;

  int dw_ = 0;
  int dh_ = 0;
  int step_ = 0;

  int pending_n_ = 0;
  int lost_n_ = 0;
  int warning_clear_n_ = 0;

  int last_image_width_ = 0;
  int last_image_height_ = 0;

  std::vector<uint8_t> depth_;
  std::vector<std::string> enabled_exit_directions_;
  P last_representative_{0.0, 0.0};
  SkeletonSignature last_signature_;
  SkeletonSignature exit_signature_;
  std::optional<Position3D> last_position_;
  std::unique_ptr<person_zone_cpp::ZoneTracker> zone_tracker_;

  std::string frame_id_ =
    "camera_depth_optical_frame";

  std::string state_ = "NONE";
  std::string pending_;
  std::string exit_direction_ = "NONE";
  std::string last_zone_before_loss_ = "NONE";
  std::string event_history_path_;

  rclcpp::Time last_kp_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_info_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PointStamped>::SharedPtr pos_pub_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr exit_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr emergency_pub_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr edge_pub_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr event_pub_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr diagnostics_pub_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr warning_ack_srv_;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  rclcpp::Subscription<
    person_pose_msgs::msg::PersonKeypoints>::SharedPtr kp_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonZoneNode>());
  rclcpp::shutdown();
  return 0;
}
