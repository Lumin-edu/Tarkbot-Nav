#ifndef TASK_FRAME_NAV__CAPTURE_STATE_HPP_
#define TASK_FRAME_NAV__CAPTURE_STATE_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "task_frame_nav/pose_math.hpp"

namespace task_frame_nav
{

// Stores the first valid body pose received after the configured startup delay,
// after applying the same fixed body->base_link transform used by Nav2 TF.
class CaptureState
{
public:
  CaptureState(double capture_delay_seconds, std::string odom_frame, std::string body_frame)
  : capture_delay_seconds_(capture_delay_seconds),
    odom_frame_(std::move(odom_frame)),
    body_frame_(std::move(body_frame))
  {
    if (!std::isfinite(capture_delay_seconds_) || capture_delay_seconds_ < 0.0) {
      throw std::invalid_argument("capture delay must be finite and non-negative");
    }
  }

  bool tryCapture(
    const nav_msgs::msg::Odometry & odometry,
    double elapsed_seconds,
    const geometry_msgs::msg::Vector3 & body_to_base_translation,
    const geometry_msgs::msg::Quaternion & body_to_base_rotation)
  {
    if (locked_ || !std::isfinite(elapsed_seconds) || elapsed_seconds < capture_delay_seconds_ ||
      odometry.header.frame_id != odom_frame_ || odometry.child_frame_id != body_frame_)
    {
      return false;
    }

    try {
      origin_.header = odometry.header;
      origin_.pose = composeBodyToBasePose(
        odometry.pose.pose, body_to_base_translation, body_to_base_rotation);
    } catch (const std::invalid_argument &) {
      return false;
    }

    locked_ = true;
    return true;
  }

  bool locked() const noexcept
  {
    return locked_;
  }

  const geometry_msgs::msg::PoseStamped & origin() const
  {
    if (!locked_) {
      throw std::logic_error("Task frame origin has not been captured");
    }
    return origin_;
  }

private:
  double capture_delay_seconds_;
  std::string odom_frame_;
  std::string body_frame_;
  bool locked_{false};
  geometry_msgs::msg::PoseStamped origin_;
};

}  // namespace task_frame_nav

#endif  // TASK_FRAME_NAV__CAPTURE_STATE_HPP_
