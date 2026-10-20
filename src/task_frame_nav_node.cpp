#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "airobot_action/action/navigate_task_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_frame_nav/capture_state.hpp"
#include "task_frame_nav/pose_math.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace task_frame_nav
{

class TaskFrameNavNode : public rclcpp::Node
{
public:
  using TaskAction = airobot_action::action::NavigateTaskPoint;
  using NavAction = nav2_msgs::action::NavigateToPose;
  using TaskGoalHandle = rclcpp_action::ServerGoalHandle<TaskAction>;
  using AutoGoalHandle = rclcpp_action::ClientGoalHandle<TaskAction>;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavAction>;

  TaskFrameNavNode()
  : Node("task_frame_nav"),
    capture_start_(std::chrono::steady_clock::now())
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    action_name_ = declare_parameter<std::string>("action_name", "task_frame_nav");
    nav_action_name_ = declare_parameter<std::string>("nav_action_name", "navigate_to_pose");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    task_frame_ = declare_parameter<std::string>("task_frame", "task_frame");
    capture_delay_ = declare_parameter<double>("capture_delay", 3.0);
    body_to_base_translation_.x = declare_parameter<double>("body_to_base_x", 0.145194959);
    body_to_base_translation_.y = declare_parameter<double>("body_to_base_y", 0.0);
    body_to_base_translation_.z = declare_parameter<double>("body_to_base_z", -0.137544261);
    body_to_base_roll_ = declare_parameter<double>("body_to_base_roll", 0.0);
    body_to_base_pitch_ = declare_parameter<double>("body_to_base_pitch", -0.812450766803360);
    body_to_base_yaw_ = declare_parameter<double>("body_to_base_yaw", 0.0);
    auto_goal_ = declare_parameter<bool>("auto_goal", false);
    auto_goal_x_ = declare_parameter<double>("goal_x", 0.0);
    auto_goal_y_ = declare_parameter<double>("goal_y", 0.0);

    if (task_frame_.empty()) {
      throw std::invalid_argument("task_frame must not be empty");
    }
    if (!std::isfinite(capture_delay_) || capture_delay_ < 0.0) {
      throw std::invalid_argument("capture_delay must be finite and non-negative");
    }
    if (!std::isfinite(body_to_base_translation_.x) ||
      !std::isfinite(body_to_base_translation_.y) ||
      !std::isfinite(body_to_base_translation_.z) ||
      !std::isfinite(body_to_base_roll_) || !std::isfinite(body_to_base_pitch_) ||
      !std::isfinite(body_to_base_yaw_))
    {
      throw std::invalid_argument("body-to-base transform parameters must be finite");
    }

    body_to_base_rotation_ = makeBodyToBaseRotation();
    capture_ = std::make_unique<CaptureState>(capture_delay_, odom_frame_, body_frame_);
    task_frame_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    nav_client_ = rclcpp_action::create_client<NavAction>(this, nav_action_name_);
    task_server_ = rclcpp_action::create_server<TaskAction>(
      this,
      action_name_,
      std::bind(&TaskFrameNavNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TaskFrameNavNode::handleCancel, this, std::placeholders::_1),
      std::bind(&TaskFrameNavNode::handleAccepted, this, std::placeholders::_1));

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&TaskFrameNavNode::odomCallback, this, std::placeholders::_1));

    if (auto_goal_) {
      if (!std::isfinite(auto_goal_x_) || !std::isfinite(auto_goal_y_)) {
        throw std::invalid_argument("goal_x and goal_y must be finite when auto_goal is enabled");
      }
      auto_goal_client_ = rclcpp_action::create_client<TaskAction>(this, action_name_);
      auto_goal_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TaskFrameNavNode::maybeSendAutomaticGoal, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Task frame navigation listening on '%s'; capturing '%s'->'%s' after %.2f s",
      action_name_.c_str(), odom_frame_.c_str(), body_frame_.c_str(), capture_delay_);
  }

private:
  static geometry_msgs::msg::Quaternion toMessage(const tf2::Quaternion & quaternion)
  {
    geometry_msgs::msg::Quaternion message;
    message.x = quaternion.x();
    message.y = quaternion.y();
    message.z = quaternion.z();
    message.w = quaternion.w();
    return message;
  }

  geometry_msgs::msg::Quaternion makeBodyToBaseRotation() const
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(body_to_base_roll_, body_to_base_pitch_, body_to_base_yaw_);
    quaternion.normalize();
    return toMessage(quaternion);
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TaskAction::Goal> goal)
  {
    if (!goal || !std::isfinite(goal->x) || !std::isfinite(goal->y)) {
      RCLCPP_WARN(get_logger(), "Rejecting task goal with non-finite x/y");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!capture_->locked()) {
      RCLCPP_WARN(get_logger(), "Rejecting task goal: Task frame is not locked yet");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal_reserved_ || active_goal_) {
      RCLCPP_WARN(get_logger(), "Rejecting task goal: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "Rejecting task goal: Nav2 NavigateToPose is unavailable");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Reserve the single active slot in the goal callback. A second callback
    // cannot pass this check while the accepted callback is being scheduled.
    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<TaskGoalHandle> goal_handle)
  {
    std::shared_ptr<NavGoalHandle> nav_goal;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!active_goal_ || active_goal_.get() != goal_handle.get()) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      cancel_requested_ = true;
      nav_goal = nav_goal_handle_;
    }

    if (nav_goal) {
      requestNavCancel(nav_goal);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<TaskGoalHandle> goal_handle)
  {
    geometry_msgs::msg::PoseStamped target;
    std::string target_error;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      active_goal_ = goal_handle;
      cancel_requested_ = false;
      try {
        const auto goal = goal_handle->get_goal();
        target = taskPointToFrame(task_frame_, goal->x, goal->y);
      } catch (const std::exception & exception) {
        target_error = exception.what();
      }
      if (target_error.empty()) {
        target.header.stamp = now();
        target.header.frame_id = task_frame_;
        target_pose_ = target;
      }
    }

    if (!target_error.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not construct Nav2 goal: %s", target_error.c_str());
      finishGoal(goal_handle, 4, target_error, false);
      return;
    }
    sendNavGoal(goal_handle, target);
  }

  void sendNavGoal(
    const std::shared_ptr<TaskGoalHandle> & task_goal,
    const geometry_msgs::msg::PoseStamped & target)
  {
    NavAction::Goal nav_goal;
    nav_goal.pose = target;

    rclcpp_action::Client<NavAction>::SendGoalOptions options;
    options.goal_response_callback =
      [this, task_goal](NavGoalHandle::SharedPtr nav_goal) {
        onNavGoalResponse(task_goal, std::move(nav_goal));
      };
    options.feedback_callback =
      [this, task_goal](
        NavGoalHandle::SharedPtr nav_goal,
        const std::shared_ptr<const NavAction::Feedback> feedback) {
        onNavFeedback(task_goal, nav_goal, feedback);
      };
    options.result_callback =
      [this, task_goal](const NavGoalHandle::WrappedResult & result) {
        onNavResult(task_goal, result);
      };

    try {
      nav_client_->async_send_goal(nav_goal, options);
    } catch (const std::exception & exception) {
      finishGoal(task_goal, 3, std::string("failed to send Nav2 goal: ") + exception.what(), false);
    }
  }

  void onNavGoalResponse(
    const std::shared_ptr<TaskGoalHandle> & task_goal,
    NavGoalHandle::SharedPtr nav_goal)
  {
    if (!nav_goal) {
      RCLCPP_WARN(get_logger(), "Nav2 rejected the NavigateToPose goal");
      finishGoal(task_goal, 3, "Nav2 rejected the NavigateToPose goal", false);
      return;
    }

    bool cancel = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!active_goal_ || active_goal_.get() != task_goal.get()) {
        return;
      }
      nav_goal_handle_ = nav_goal;
      cancel = cancel_requested_;
    }
    if (cancel) {
      requestNavCancel(nav_goal);
    }
  }

  void requestNavCancel(const NavGoalHandle::SharedPtr & nav_goal)
  {
    try {
      nav_client_->async_cancel_goal(nav_goal);
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Unable to request Nav2 cancellation: %s", exception.what());
    }
  }

  void onNavFeedback(
    const std::shared_ptr<TaskGoalHandle> & task_goal,
    const NavGoalHandle::SharedPtr & nav_goal,
    const std::shared_ptr<const NavAction::Feedback> & nav_feedback)
  {
    if (!nav_feedback) {
      return;
    }
    std::shared_ptr<TaskGoalHandle> active;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!active_goal_ || active_goal_.get() != task_goal.get() || !active_goal_->is_active()) {
        return;
      }
      active = active_goal_;
    }

    auto feedback = std::make_shared<TaskAction::Feedback>();
    const auto nav_status = nav_goal ? nav_goal->get_status() :
      action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    feedback->status = static_cast<uint8_t>(nav_status);
    switch (nav_status) {
      case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
        feedback->status_text = "accepted";
        break;
      case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        feedback->status_text = "executing";
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELING:
        feedback->status_text = "canceling";
        break;
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        feedback->status_text = "succeeded";
        break;
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        feedback->status_text = "canceled";
        break;
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        feedback->status_text = "aborted";
        break;
      default:
        feedback->status_text = "unknown";
        break;
    }
    feedback->current_pose = nav_feedback->current_pose;
    feedback->distance_remaining = nav_feedback->distance_remaining;
    feedback->navigation_time = nav_feedback->navigation_time;
    feedback->estimated_time_remaining = nav_feedback->estimated_time_remaining;
    feedback->number_of_recoveries = static_cast<uint16_t>(std::max<int16_t>(
        0, nav_feedback->number_of_recoveries));
    try {
      active->publish_feedback(feedback);
    } catch (const std::exception &) {
      // A terminal result may race with the last Nav2 feedback callback.
    }
  }

  void onNavResult(
    const std::shared_ptr<TaskGoalHandle> & task_goal,
    const NavGoalHandle::WrappedResult & wrapped_result)
  {
    bool cancel_requested = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!active_goal_ || active_goal_.get() != task_goal.get()) {
        return;
      }
      cancel_requested = cancel_requested_;
    }

    if (cancel_requested) {
      finishGoal(task_goal, 2, "task goal canceled", true);
      return;
    }

    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        finishGoal(task_goal, 0, "Nav2 reached the target", false);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        finishGoal(task_goal, 2, "Nav2 canceled the target", false);
        break;
      case rclcpp_action::ResultCode::ABORTED:
      default:
        finishGoal(task_goal, 1, "Nav2 aborted the target", false);
        break;
    }
  }

  void finishGoal(
    const std::shared_ptr<TaskGoalHandle> & task_goal,
    int32_t error_code,
    const std::string & message,
    bool canceled_by_client)
  {
    geometry_msgs::msg::PoseStamped target;
    bool should_finish = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!active_goal_ || active_goal_.get() != task_goal.get()) {
        return;
      }
      target = target_pose_;
      active_goal_.reset();
      nav_goal_handle_.reset();
      goal_reserved_ = false;
      cancel_requested_ = false;
      should_finish = true;
    }
    if (!should_finish) {
      return;
    }

    auto result = std::make_shared<TaskAction::Result>();
    result->success = error_code == 0;
    result->error_code = error_code;
    result->message = message;
    result->target_pose = target;
    try {
      if (canceled_by_client) {
        task_goal->canceled(result);
      } else if (error_code == 0) {
        task_goal->succeed(result);
      } else {
        task_goal->abort(result);
      }
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Unable to publish task action result: %s", exception.what());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (!message) {
      return;
    }
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - capture_start_).count();
    bool locked_now = false;
    bool have_task_transform = false;
    geometry_msgs::msg::TransformStamped task_transform;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      try {
        if (message->header.frame_id != odom_frame_ ||
          message->child_frame_id != body_frame_)
        {
          return;
        }

        locked_now = capture_->tryCapture(
          *message, elapsed, body_to_base_translation_, body_to_base_rotation_);

        // task_frame is established once from the compensated base pose at
        // the capture instant.  It is a fixed horizontal planning frame, so
        // publish it on /tf_static rather than repeatedly adding old samples
        // to the dynamic TF cache.  Replaying the capture transform with
        // FAST-LIO timestamps causes message filters to reject point clouds
        // whose timestamp predates the first dynamic sample.
        if (locked_now) {
          task_transform = makeTaskFrameTransform(capture_->origin(), task_frame_);
          have_task_transform = true;
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Ignoring FAST-LIO odometry: %s", exception.what());
        return;
      }
      latest_odom_ = *message;
      have_odom_ = true;
    }
    if (have_task_transform) {
      task_frame_broadcaster_->sendTransform(task_transform);
    }
    if (locked_now) {
      RCLCPP_INFO(
        get_logger(),
        "Task frame '%s' locked to base_link at %.3f s: origin (%.3f, %.3f, %.3f)",
        task_frame_.c_str(), elapsed, task_transform.transform.translation.x,
        task_transform.transform.translation.y, task_transform.transform.translation.z);
    }
  }

  void maybeSendAutomaticGoal()
  {
    if (!auto_goal_client_ || !nav_client_->action_server_is_ready()) {
      return;
    }
    const auto now_steady = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!capture_->locked() || goal_reserved_ || active_goal_ || auto_goal_sent_ ||
        auto_goal_pending_ || now_steady < auto_goal_next_attempt_)
      {
        return;
      }
    }
    if (!auto_goal_client_->action_server_is_ready()) {
      return;
    }
    TaskAction::Goal goal;
    goal.x = auto_goal_x_;
    goal.y = auto_goal_y_;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto_goal_pending_ = true;
    }

    rclcpp_action::Client<TaskAction>::SendGoalOptions options;
    options.goal_response_callback = [this](AutoGoalHandle::SharedPtr goal_handle) {
      if (!goal_handle) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto_goal_pending_ = false;
        auto_goal_sent_ = false;
        auto_goal_next_attempt_ = std::chrono::steady_clock::now() + auto_goal_retry_delay_;
        RCLCPP_WARN(
          get_logger(),
          "Automatic task goal was rejected; retrying after Nav2 becomes active");
        return;
      }
      RCLCPP_INFO(get_logger(), "Automatic task goal accepted by task action server");
    };
    options.result_callback = [this](const AutoGoalHandle::WrappedResult & result) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto_goal_pending_ = false;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        auto_goal_sent_ = true;
        RCLCPP_INFO(get_logger(), "Automatic task goal completed successfully");
      } else {
        auto_goal_sent_ = false;
        auto_goal_next_attempt_ = std::chrono::steady_clock::now() + auto_goal_retry_delay_;
        RCLCPP_WARN(
          get_logger(),
          "Automatic task goal ended with result code %d; retrying",
          static_cast<int>(result.code));
      }
    };

    try {
      auto_goal_client_->async_send_goal(goal, options);
      RCLCPP_INFO(get_logger(), "Submitted automatic Task goal (%.3f, %.3f)", goal.x, goal.y);
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto_goal_pending_ = false;
      auto_goal_sent_ = false;
      auto_goal_next_attempt_ = std::chrono::steady_clock::now() + auto_goal_retry_delay_;
      RCLCPP_WARN(
        get_logger(), "Unable to submit automatic Task goal: %s; retrying", exception.what());
    }
  }

  std::string odom_topic_;
  std::string action_name_;
  std::string nav_action_name_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string task_frame_;
  double capture_delay_{3.0};
  geometry_msgs::msg::Vector3 body_to_base_translation_;
  double body_to_base_roll_{0.0};
  double body_to_base_pitch_{-M_PI / 4.0};
  double body_to_base_yaw_{0.0};
  geometry_msgs::msg::Quaternion body_to_base_rotation_;
  bool auto_goal_{false};
  double auto_goal_x_{0.0};
  double auto_goal_y_{0.0};
  bool auto_goal_sent_{false};
  bool auto_goal_pending_{false};
  const std::chrono::milliseconds auto_goal_retry_delay_{500};
  std::chrono::steady_clock::time_point auto_goal_next_attempt_{};

  std::chrono::steady_clock::time_point capture_start_;
  std::unique_ptr<CaptureState> capture_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> task_frame_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::Server<TaskAction>::SharedPtr task_server_;
  rclcpp_action::Client<TaskAction>::SharedPtr auto_goal_client_;
  rclcpp::TimerBase::SharedPtr auto_goal_timer_;

  std::mutex state_mutex_;
  bool goal_reserved_{false};
  bool cancel_requested_{false};
  bool have_odom_{false};
  nav_msgs::msg::Odometry latest_odom_;
  std::shared_ptr<TaskGoalHandle> active_goal_;
  std::shared_ptr<NavGoalHandle> nav_goal_handle_;
  geometry_msgs::msg::PoseStamped target_pose_;
};

}  // namespace task_frame_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<task_frame_nav::TaskFrameNavNode>());
  rclcpp::shutdown();
  return 0;
}
