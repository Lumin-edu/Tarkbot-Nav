#include <cmath>

#include "gtest/gtest.h"
#include "task_frame_nav/pose_math.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion makeQuaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

void expectNear(double actual, double expected, double tolerance = 1e-6)
{
  EXPECT_NEAR(actual, expected, tolerance);
}
}  // namespace

TEST(PoseMath, BodyToBasePitchCompensationProducesPlanningPlane)
{
  geometry_msgs::msg::Pose body_pose;
  body_pose.position.x = 2.0;
  body_pose.position.y = -1.0;
  body_pose.position.z = 1.0;
  body_pose.orientation = makeQuaternion(0.0, kPi / 4.0, 0.7);

  const auto base_pose = task_frame_nav::composeBodyToBasePose(
    body_pose, geometry_msgs::msg::Vector3{}, makeQuaternion(0.0, -kPi / 4.0, 0.0));

  tf2::Quaternion base_q(
    base_pose.orientation.x, base_pose.orientation.y,
    base_pose.orientation.z, base_pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(base_q).getRPY(roll, pitch, yaw);
  expectNear(roll, 0.0);
  expectNear(pitch, 0.0);
  expectNear(yaw, 0.7);

  geometry_msgs::msg::PoseStamped captured_base;
  captured_base.header.frame_id = "camera_init";
  captured_base.pose = base_pose;
  const auto task_tf = task_frame_nav::makeTaskFrameTransform(captured_base, "task_frame");
  tf2::Quaternion task_q(
    task_tf.transform.rotation.x, task_tf.transform.rotation.y,
    task_tf.transform.rotation.z, task_tf.transform.rotation.w);
  tf2::Vector3 task_up = tf2::quatRotate(task_q, tf2::Vector3(0.0, 0.0, 1.0));
  expectNear(task_up.x(), 0.0);
  expectNear(task_up.y(), 0.0);
  expectNear(task_up.z(), 1.0);
}

TEST(PoseMath, BodyToBaseTranslationUsesBodyOrientation)
{
  geometry_msgs::msg::Pose body_pose;
  body_pose.position.x = 1.0;
  body_pose.position.y = 2.0;
  body_pose.position.z = 0.5;
  body_pose.orientation = makeQuaternion(0.0, 0.0, kPi / 2.0);

  geometry_msgs::msg::Vector3 translation;
  translation.x = 0.2;
  translation.z = -0.1;
  const auto base = task_frame_nav::composeBodyToBasePose(
    body_pose, translation, makeQuaternion(0.0, 0.0, 0.0));

  expectNear(base.position.x, 1.0);
  expectNear(base.position.y, 2.2);
  expectNear(base.position.z, 0.4);
}

TEST(PoseMath, TaskFrameKeepsCapturedBasePlaneOrientation)
{
  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.frame_id = "camera_init";
  base_pose.header.stamp.sec = 8;
  base_pose.header.stamp.nanosec = 9;
  base_pose.pose.position.x = 1.0;
  base_pose.pose.position.y = 2.0;
  base_pose.pose.position.z = 3.0;
  base_pose.pose.orientation = makeQuaternion(0.1, -0.2, 0.7);

  const auto task_tf = task_frame_nav::makeTaskFrameTransform(
    base_pose, "task_frame");

  EXPECT_EQ(task_tf.header.frame_id, "camera_init");
  EXPECT_EQ(task_tf.child_frame_id, "task_frame");
  EXPECT_EQ(task_tf.header.stamp.sec, base_pose.header.stamp.sec);
  EXPECT_NEAR(task_tf.transform.translation.x, 1.0, 1e-6);
  EXPECT_NEAR(task_tf.transform.translation.y, 2.0, 1e-6);
  EXPECT_NEAR(task_tf.transform.translation.z, 3.0, 1e-6);
  EXPECT_NEAR(task_tf.transform.rotation.x, base_pose.pose.orientation.x, 1e-6);
  EXPECT_NEAR(task_tf.transform.rotation.y, base_pose.pose.orientation.y, 1e-6);
  EXPECT_NEAR(task_tf.transform.rotation.z, base_pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(task_tf.transform.rotation.w, base_pose.pose.orientation.w, 1e-6);
}

TEST(PoseMath, TaskPointUsesCapturedBaseAxes)
{
  const auto target = task_frame_nav::taskPointToFrame("task_frame", 1.0, -2.0);

  EXPECT_EQ(target.header.frame_id, "task_frame");
  EXPECT_NEAR(target.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(target.pose.position.y, -2.0, 1e-6);
  EXPECT_NEAR(target.pose.position.z, 0.0, 1e-6);
  EXPECT_NEAR(target.pose.orientation.x, 0.0, 1e-6);
  EXPECT_NEAR(target.pose.orientation.y, 0.0, 1e-6);
  EXPECT_NEAR(target.pose.orientation.z, 0.0, 1e-6);
  EXPECT_NEAR(target.pose.orientation.w, 1.0, 1e-6);
}
