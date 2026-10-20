#include <cmath>

#include "gtest/gtest.h"
#include "task_frame_nav/capture_state.hpp"

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
}  // namespace

TEST(CaptureState, WaitsUntilDelayAndThenLocksComposedBasePoseOnce)
{
  task_frame_nav::CaptureState capture(3.0, "camera_init", "body");
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "camera_init";
  odometry.child_frame_id = "body";
  odometry.pose.pose.position.x = 2.0;
  odometry.pose.pose.position.y = -1.0;
  odometry.pose.pose.position.z = 0.4;
  odometry.pose.pose.orientation = makeQuaternion(0.0, kPi / 4.0, 0.7);
  geometry_msgs::msg::Vector3 translation;
  geometry_msgs::msg::Quaternion leveling = makeQuaternion(0.0, -kPi / 4.0, 0.0);

  EXPECT_FALSE(capture.tryCapture(odometry, 2.99, translation, leveling));
  EXPECT_FALSE(capture.locked());
  ASSERT_TRUE(capture.tryCapture(odometry, 3.0, translation, leveling));
  ASSERT_TRUE(capture.locked());
  const auto locked_pose = capture.origin();

  odometry.pose.pose.position.x = 99.0;
  odometry.pose.pose.orientation = makeQuaternion(0.0, 0.0, -1.0);
  EXPECT_FALSE(capture.tryCapture(odometry, 10.0, translation, leveling));
  EXPECT_DOUBLE_EQ(capture.origin().pose.position.x, locked_pose.pose.position.x);
}

TEST(CaptureState, RejectsOdometryWithUnexpectedFrames)
{
  task_frame_nav::CaptureState capture(0.0, "camera_init", "body");
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.orientation.w = 1.0;

  EXPECT_FALSE(capture.tryCapture(
      odometry, 1.0, geometry_msgs::msg::Vector3{}, geometry_msgs::msg::Quaternion{}));
  EXPECT_FALSE(capture.locked());
}

TEST(CaptureState, InvalidQuaternionDoesNotLockAndCanRecoverOnNextSample)
{
  task_frame_nav::CaptureState capture(0.0, "camera_init", "body");
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "camera_init";
  odometry.child_frame_id = "body";
  odometry.pose.pose.orientation.x = 0.0;
  odometry.pose.pose.orientation.y = 0.0;
  odometry.pose.pose.orientation.z = 0.0;
  odometry.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(capture.tryCapture(
      odometry, 1.0, geometry_msgs::msg::Vector3{}, makeQuaternion(0.0, 0.0, 0.0)));
  EXPECT_FALSE(capture.locked());

  odometry.pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(capture.tryCapture(
      odometry, 1.0, geometry_msgs::msg::Vector3{}, makeQuaternion(0.0, 0.0, 0.0)));
  EXPECT_TRUE(capture.locked());
}

TEST(CaptureState, LocksComposedBasePoseWithoutChangingItsTilt)
{
  task_frame_nav::CaptureState capture(0.0, "camera_init", "body");
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "camera_init";
  odometry.child_frame_id = "body";
  odometry.pose.pose.orientation = makeQuaternion(0.25, -0.30, 0.40);

  ASSERT_TRUE(capture.tryCapture(
      odometry, 0.0, geometry_msgs::msg::Vector3{}, makeQuaternion(0.0, 0.0, 0.0)));

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(task_frame_nav::normalizedQuaternion(capture.origin().pose.orientation)).getRPY(
    roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.25, 1e-6);
  EXPECT_NEAR(pitch, -0.30, 1e-6);
  EXPECT_NEAR(yaw, 0.40, 1e-6);
}
