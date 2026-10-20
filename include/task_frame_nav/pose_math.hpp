#ifndef TASK_FRAME_NAV__POSE_MATH_HPP_
#define TASK_FRAME_NAV__POSE_MATH_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace task_frame_nav
{

inline tf2::Quaternion normalizedQuaternion(const geometry_msgs::msg::Quaternion & message)
{
  tf2::Quaternion quaternion(
    message.x, message.y, message.z, message.w);
  const double norm = quaternion.length();
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("pose contains an invalid quaternion");
  }
  quaternion /= norm;
  return quaternion;
}

inline geometry_msgs::msg::Quaternion toMessage(const tf2::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

// Compose the native camera_init->body pose with the static body->base_link transform.
inline geometry_msgs::msg::Pose composeBodyToBasePose(
  const geometry_msgs::msg::Pose & body_pose,
  const geometry_msgs::msg::Vector3 & body_to_base_translation,
  const geometry_msgs::msg::Quaternion & body_to_base_rotation)
{
  if (!std::isfinite(body_pose.position.x) ||
    !std::isfinite(body_pose.position.y) ||
    !std::isfinite(body_pose.position.z) ||
    !std::isfinite(body_to_base_translation.x) ||
    !std::isfinite(body_to_base_translation.y) ||
    !std::isfinite(body_to_base_translation.z))
  {
    throw std::invalid_argument("pose translation contains a non-finite value");
  }

  const auto body_rotation = normalizedQuaternion(body_pose.orientation);
  const auto base_rotation = normalizedQuaternion(body_to_base_rotation);
  const tf2::Vector3 body_position(
    body_pose.position.x, body_pose.position.y, body_pose.position.z);
  const tf2::Vector3 offset_body(
    body_to_base_translation.x,
    body_to_base_translation.y,
    body_to_base_translation.z);
  const tf2::Vector3 base_position = body_position + tf2::quatRotate(body_rotation, offset_body);

  tf2::Quaternion odom_to_base_rotation = body_rotation * base_rotation;
  odom_to_base_rotation.normalize();

  geometry_msgs::msg::Pose result;
  result.position.x = base_position.x();
  result.position.y = base_position.y();
  result.position.z = base_position.z();
  result.orientation = toMessage(odom_to_base_rotation);
  return result;
}

// Anchor a fixed task frame to the captured base_link pose.  Preserve the full
// composed orientation so camera_init->task_frame and body->base_link describe
// the same plane. The mount compensation is applied by body->base_link before
// this pose is captured; no separate point-cloud rotation is performed.
inline geometry_msgs::msg::TransformStamped makeTaskFrameTransform(
  const geometry_msgs::msg::PoseStamped & base_pose,
  const std::string & task_frame)
{
  if (base_pose.header.frame_id.empty() || task_frame.empty() ||
    !std::isfinite(base_pose.pose.position.x) ||
    !std::isfinite(base_pose.pose.position.y) ||
    !std::isfinite(base_pose.pose.position.z))
  {
    throw std::invalid_argument("task frame anchor contains invalid data");
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header = base_pose.header;
  transform.child_frame_id = task_frame;
  transform.transform.translation.x = base_pose.pose.position.x;
  transform.transform.translation.y = base_pose.pose.position.y;
  transform.transform.translation.z = base_pose.pose.position.z;
  transform.transform.rotation = toMessage(normalizedQuaternion(base_pose.pose.orientation));
  return transform;
}

// Express a task goal in the fixed frame anchored to the captured base. The
// frame already carries the captured base orientation, and z is zero because
// Nav2 plans in that plane.
inline geometry_msgs::msg::PoseStamped taskPointToFrame(
  const std::string & task_frame,
  double task_x,
  double task_y)
{
  if (task_frame.empty() || !std::isfinite(task_x) || !std::isfinite(task_y)) {
    throw std::invalid_argument("task target contains invalid frame or coordinates");
  }

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = task_frame;
  target.pose.position.x = task_x;
  target.pose.position.y = task_y;
  target.pose.position.z = 0.0;
  target.pose.orientation.w = 1.0;
  return target;
}

}  // namespace task_frame_nav

#endif  // TASK_FRAME_NAV__POSE_MATH_HPP_
