#include "testAMR7/omni_base_controller.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace omni_base_controller
{

OmniBaseController::OmniBaseController()
: controller_interface::ControllerInterface(),
  wheel_radius_(0.0275),
  wheel_base_(0.13773),
  wheel_track_(0.085354),
  max_linear_velocity_(2.0),
  max_angular_velocity_(2.0),
  max_wheel_velocity_(50.0),
  max_steer_velocity_(10.0),
  cmd_vel_timeout_(1.0),
  steer_angle_tolerance_(0.01),
  use_shortest_path_(true),
  enable_odom_tf_(false),
  enable_odometry_(false),
  publish_rate_(50.0),
  steering_threshold_(0.2),
  velocity_scale_factor_(0.0),
  odom_frame_id_("odom"),
  base_frame_id_("base_link"),
  odom_x_(0.0),
  odom_y_(0.0),
  odom_theta_(0.0)
{
  // Initialize wheel modules with correct positions
  wheel_modules_[0] = { 0.068865,  0.042677, 0.0, 0.0, 0.0 }; // BR
  wheel_modules_[1] = { -0.068865, 0.042677, 0.0, 0.0, 0.0 }; // FR
  wheel_modules_[2] = { -0.068865, -0.042677, 0.0, 0.0, 0.0 }; // FL
  wheel_modules_[3] = { 0.068865, -0.042677, 0.0, 0.0, 0.0 }; // BL
}

controller_interface::CallbackReturn OmniBaseController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("wheel_joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("steer_joints", std::vector<std::string>());
    auto_declare<double>("wheel_radius", 0.0275);
    auto_declare<double>("wheel_base", 0.13773);
    auto_declare<double>("wheel_track", 0.085354);
    auto_declare<double>("cmd_vel_timeout", 1.0);
    auto_declare<double>("max_linear_velocity", 0.5);
    auto_declare<double>("max_angular_velocity", 2.0);
    auto_declare<double>("max_wheel_velocity", 50.0);
    auto_declare<double>("max_steer_velocity", 10.0);
    auto_declare<double>("steer_angle_tolerance", 0.01);
    auto_declare<bool>("use_shortest_path", true);
    auto_declare<bool>("enable_odometry", false);
    auto_declare<bool>("enable_odom_tf", false);
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<std::string>("base_frame_id", "base_link");
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<double>("steering_threshold", 0.2);
    auto_declare<double>("velocity_scale_factor", 0.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  wheel_joint_names_ = get_node()->get_parameter("wheel_joints").as_string_array();
  steer_joint_names_ = get_node()->get_parameter("steer_joints").as_string_array();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  wheel_base_ = get_node()->get_parameter("wheel_base").as_double();
  wheel_track_ = get_node()->get_parameter("wheel_track").as_double();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  max_wheel_velocity_ = get_node()->get_parameter("max_wheel_velocity").as_double();
  max_steer_velocity_ = get_node()->get_parameter("max_steer_velocity").as_double();
  steer_angle_tolerance_ = get_node()->get_parameter("steer_angle_tolerance").as_double();
  use_shortest_path_ = get_node()->get_parameter("use_shortest_path").as_bool();
  enable_odometry_ = get_node()->get_parameter("enable_odometry").as_bool();
  enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();
  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  steering_threshold_ = get_node()->get_parameter("steering_threshold").as_double();
  velocity_scale_factor_ = get_node()->get_parameter("velocity_scale_factor").as_double();

  if (wheel_joint_names_.size() != 4 || steer_joint_names_.size() != 4)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Must have exactly 4 wheel and 4 steer joints");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) { cmdVelCallback(msg); });

  if (enable_odometry_)
  {
    odom_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS()));

    if (enable_odom_tf_)
    {
      tf_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS()));
    }
  }

  RCLCPP_INFO(get_node()->get_logger(), "OmniBaseController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
OmniBaseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Wheel joints: VELOCITY
  for (const auto & joint_name : wheel_joint_names_)
  {
    config.names.push_back(joint_name + "/velocity");
  }
  // Steering joints: POSITION
  for (const auto & joint_name : steer_joint_names_)
  {
    config.names.push_back(joint_name + "/position");
  }

  return config;
}

controller_interface::InterfaceConfiguration 
OmniBaseController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Wheel: position, velocity
  for (const auto & joint_name : wheel_joint_names_)
  {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }
  // Steering: position, velocity
  for (const auto & joint_name : steer_joint_names_)
  {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn OmniBaseController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (enable_odometry_)
  {
    odom_x_ = 0.0;
    odom_y_ = 0.0;
    odom_theta_ = 0.0;
    last_odom_time_ = get_node()->now();
  }
  
  last_cmd_time_ = get_node()->now();
  received_velocity_msg_ptr_.writeFromNonRT(nullptr);
  
  // Initialize previous steering angles from current state
  for (size_t i = 0; i < 4; ++i)
  {
    auto current_angle_opt = state_interfaces_[8 + i * 2].get_optional();
    if (current_angle_opt.has_value())
    {
      wheel_modules_[i].steer_angle_prev = current_angle_opt.value();
    }
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "OmniBaseController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniBaseController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop wheels and reset steering (safe)
  for (size_t i = 0; i < 4; ++i)
  {
    [[maybe_unused]] bool wheel_ok = command_interfaces_[i].set_value(0.0);
    [[maybe_unused]] bool steer_ok = command_interfaces_[i + 4].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "OmniBaseController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmniBaseController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto current_cmd = received_velocity_msg_ptr_.readFromRT();
  double vx = 0.0, vy = 0.0, omega = 0.0;

  if (current_cmd && *current_cmd)
  {
    double dt = (time - last_cmd_time_).seconds();
    if (dt <= cmd_vel_timeout_)
    {
      vx = std::clamp((*current_cmd)->linear.x, -max_linear_velocity_, max_linear_velocity_);
      vy = std::clamp((*current_cmd)->linear.y, -max_linear_velocity_, max_linear_velocity_);
      omega = std::clamp((*current_cmd)->angular.z, -max_angular_velocity_, max_angular_velocity_);
    }
  }

  std::array<double, 4> wheel_velocities;
  std::array<double, 4> steer_angles;

  computeInverseKinematics(vx, vy, omega, wheel_velocities, steer_angles);

  if (use_shortest_path_)
  {
    optimizeSteeringAngles(wheel_velocities, steer_angles);
  }

  // CRITICAL: Check if ANY wheel needs significant steering adjustment
  bool any_wheel_needs_steering = false;
  double max_steering_error = 0.0;
  
  for (size_t i = 0; i < 4; ++i)
  {
    auto current_steer_opt = state_interfaces_[8 + i * 2].get_optional();
    if (current_steer_opt.has_value())
    {
      double current_steer = current_steer_opt.value();
      double angle_error = std::abs(normalizeAngle(steer_angles[i] - current_steer));
      max_steering_error = std::max(max_steering_error, angle_error);
      
      if (angle_error > steering_threshold_)
      {
        any_wheel_needs_steering = true;
      }
    }
  }

  // If ANY wheel needs steering, stop ALL wheels to prevent body motion
  if (any_wheel_needs_steering)
  {
    RCLCPP_DEBUG_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 500,
      "Steering in progress (max error: %.3f rad), wheels stopped", max_steering_error);
    
    // Set all wheel velocities to zero (or scaled value)
    for (size_t i = 0; i < 4; ++i)
    {
      wheel_velocities[i] *= velocity_scale_factor_;
    }
  }
  else
  {
    // All wheels are aligned, apply gentle cosine scaling for smoothness
    for (size_t i = 0; i < 4; ++i)
    {
      auto current_steer_opt = state_interfaces_[8 + i * 2].get_optional();
      if (current_steer_opt.has_value())
      {
        double current_steer = current_steer_opt.value();
        double angle_error = normalizeAngle(steer_angles[i] - current_steer);
        double cos_scale = std::cos(angle_error);
        
        // Gentle cubic scaling, with minimum 20% to avoid stalling
        double scale_factor = std::max(cos_scale * cos_scale * cos_scale, 0.2);
        wheel_velocities[i] *= scale_factor;
      }
    }
  }

  // Send commands: wheels = velocity, steering = position
  for (size_t i = 0; i < 4; ++i)
  {
    [[maybe_unused]] bool wheel_ok = command_interfaces_[i].set_value(wheel_velocities[i]);
    [[maybe_unused]] bool steer_ok = command_interfaces_[i + 4].set_value(steer_angles[i]);
  }

  if (enable_odometry_)
  {
    updateOdometry(time, period);
    publishOdometry(time);
  }

  return controller_interface::return_type::OK;
}

void OmniBaseController::cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  if (!msg) return;
  
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 30,
    "cmd_vel: vx=%.3f vy=%.3f omega=%.3f", msg->linear.x, msg->linear.y, msg->angular.z);
  
  received_velocity_msg_ptr_.writeFromNonRT(msg);
  last_cmd_time_ = get_node()->now();
}

void OmniBaseController::computeInverseKinematics(
  double vx, double vy, double omega,
  std::array<double, 4> & wheel_velocities,
  std::array<double, 4> & steer_angles)
{
  for (size_t i = 0; i < 4; ++i)
  {
    // Compute wheel velocity components in body frame
    double vx_wheel = vx - omega * wheel_modules_[i].y;
    double vy_wheel = vy + omega * wheel_modules_[i].x;
    double wheel_speed = std::hypot(vx_wheel, vy_wheel);
    
    // Restore negative sign as per your configuration
    double angle = -std::atan2(vy_wheel, vx_wheel);

    if (wheel_speed < 1e-6)
    {
      // If not moving, maintain current steering angle
      wheel_velocities[i] = 0.0;
      steer_angles[i] = wheel_modules_[i].steer_angle_prev;
    }
    else
    {
      steer_angles[i] = angle;
      wheel_velocities[i] = wheel_speed / wheel_radius_;

      if (std::abs(wheel_velocities[i]) > max_wheel_velocity_)
      {
        wheel_velocities[i] = std::copysign(max_wheel_velocity_, wheel_velocities[i]);
      }
    }
  }
}

void OmniBaseController::optimizeSteeringAngles(
  std::array<double, 4> & wheel_velocities,
  std::array<double, 4> & steer_angles)
{
  for (size_t i = 0; i < 4; ++i)
  {
    // Skip optimization if wheel is stationary
    if (std::abs(wheel_velocities[i]) < 1e-6)
    {
      // Keep previous angle when stopped
      steer_angles[i] = wheel_modules_[i].steer_angle_prev;
      continue;
    }
    
    // Get the actual current steering angle from hardware state
    auto current_angle_opt = state_interfaces_[8 + i * 2].get_optional();
    if (!current_angle_opt.has_value())
    {
      // If we can't read state, keep the target angle
      wheel_modules_[i].steer_angle_prev = steer_angles[i];
      continue;
    }
    
    double current_angle = current_angle_opt.value();
    double target_angle = steer_angles[i];
    
    // Normalize both angles to [-π, π] for fair comparison
    current_angle = normalizeAngle(current_angle);
    target_angle = normalizeAngle(target_angle);
    
    // Calculate the direct angular difference to target
    double direct_diff = normalizeAngle(target_angle - current_angle);
    double direct_rotation = std::abs(direct_diff);
    
    // Calculate the difference if we flip 180° and reverse wheel
    double flipped_target = normalizeAngle(target_angle + M_PI);
    double flipped_diff = normalizeAngle(flipped_target - current_angle);
    double flipped_rotation = std::abs(flipped_diff);
    
    // Choose the path that requires LESS rotation
    // CRITICAL: Use strict inequality to prefer direct path when equal
    if (flipped_rotation < direct_rotation)
    {
      // Flipped path is strictly shorter
      steer_angles[i] = flipped_target;
      wheel_velocities[i] = -wheel_velocities[i];
      
      RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Wheel %zu: Flipping (%.1f° vs %.1f°)", 
        i, direct_rotation * 180.0 / M_PI, flipped_rotation * 180.0 / M_PI);
    }
    else
    {
      // Direct path is shorter or equal - use it
      steer_angles[i] = target_angle;
      // Keep wheel_velocities[i] as is
    }
    
    // Update the stored previous angle for next iteration
    wheel_modules_[i].steer_angle_prev = steer_angles[i];
  }
}

double OmniBaseController::normalizeAngle(double angle)
{
  // Normalize to [-π, π]
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool OmniBaseController::needsSteeringAdjustment(const std::array<double, 4> & target_angles)
{
  for (size_t i = 0; i < 4; ++i)
  {
    // Get current steering angle from state interface
    auto current_angle_opt = state_interfaces_[8 + i * 2].get_optional();
    if (!current_angle_opt.has_value()) continue;
    
    double current_angle = current_angle_opt.value();
    double angle_diff = std::abs(normalizeAngle(target_angles[i] - current_angle));
    
    // If any wheel needs to turn more than threshold, enable steering priority
    if (angle_diff > steering_threshold_)
    {
      return true;
    }
  }
  return false;
}

void OmniBaseController::updateOdometry(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!enable_odometry_) return;

  double dt = period.seconds();
  if (dt < 1e-6) return;

  double vx_sum = 0.0, vy_sum = 0.0;
  double omega_sum = 0.0;
  int valid_wheels = 0;

  for (size_t i = 0; i < 4; ++i)
  {
    auto wheel_vel_opt = state_interfaces_[i * 2 + 1].get_optional();
    auto steer_pos_opt = state_interfaces_[8 + i * 2].get_optional();

    if (!wheel_vel_opt.has_value() || !steer_pos_opt.has_value()) continue;

    double wheel_angular_vel = wheel_vel_opt.value();
    double steer_angle = steer_pos_opt.value();
    double x = wheel_modules_[i].x;
    double y = wheel_modules_[i].y;

    // Convert wheel angular velocity to linear velocity
    double linear_vel = wheel_angular_vel * wheel_radius_;

    // Decompose into body-frame components
    double vx_wheel = linear_vel * std::cos(steer_angle);
    double vy_wheel = linear_vel * std::sin(steer_angle);

    vx_sum += vx_wheel;
    vy_sum += vy_wheel;

    // Angular velocity contribution from this wheel
    double r_squared = x * x + y * y;
    if (r_squared > 1e-6)
    {
      omega_sum += (x * vy_wheel - y * vx_wheel) / r_squared;
      valid_wheels++;
    }
  }

  if (valid_wheels == 0) return;

  // Average the velocities
  double vx_body = vx_sum / valid_wheels;
  double vy_body = vy_sum / valid_wheels;
  double omega_body = omega_sum / valid_wheels;

  // Apply simple low-pass filter to reduce noise
  const double alpha = 0.3;
  vx_body = alpha * vx_body + (1.0 - alpha) * last_vx_body_;
  vy_body = alpha * vy_body + (1.0 - alpha) * last_vy_body_;
  omega_body = alpha * omega_body + (1.0 - alpha) * last_omega_body_;
  
  last_vx_body_ = vx_body;
  last_vy_body_ = vy_body;
  last_omega_body_ = omega_body;

  // Update orientation first
  odom_theta_ += omega_body * dt;
  odom_theta_ = normalizeAngle(odom_theta_);

  // Transform body velocities to world frame
  double cos_theta = std::cos(odom_theta_);
  double sin_theta = std::sin(odom_theta_);
  
  double vx_world = vx_body * cos_theta - vy_body * sin_theta;
  double vy_world = vx_body * sin_theta + vy_body * cos_theta;

  // Integrate position in world frame
  odom_x_ += vx_world * dt;
  odom_y_ += vy_world * dt;

  RCLCPP_DEBUG_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "Odom: x=%.3f y=%.3f θ=%.3f | vx=%.3f vy=%.3f ω=%.3f",
    odom_x_, odom_y_, odom_theta_, vx_body, vy_body, omega_body);
}

void OmniBaseController::publishOdometry(const rclcpp::Time & time)
{
  if (!enable_odometry_) return;

  // Compute current body-frame velocities from wheel states
  double vx_body = 0.0, vy_body = 0.0, omega_body = 0.0;
  int valid_wheels = 0;

  for (size_t i = 0; i < 4; ++i)
  {
    auto wheel_vel_opt = state_interfaces_[i * 2 + 1].get_optional();
    auto steer_pos_opt = state_interfaces_[8 + i * 2].get_optional();

    if (!wheel_vel_opt.has_value() || !steer_pos_opt.has_value()) continue;

    double linear_vel = wheel_vel_opt.value() * wheel_radius_;
    double steer_angle = steer_pos_opt.value();
    
    double vx_wheel = linear_vel * std::cos(steer_angle);
    double vy_wheel = linear_vel * std::sin(steer_angle);
    
    vx_body += vx_wheel;
    vy_body += vy_wheel;

    double x = wheel_modules_[i].x;
    double y = wheel_modules_[i].y;
    double r_squared = x * x + y * y;
    if (r_squared > 1e-6)
    {
      omega_body += (x * vy_wheel - y * vx_wheel) / r_squared;
      valid_wheels++;
    }
  }

  if (valid_wheels > 0)
  {
    vx_body /= valid_wheels;
    vy_body /= valid_wheels;
    omega_body /= valid_wheels;
  }

  // Publish odometry message
  if (odom_publisher_ && odom_publisher_->trylock())
  {
    auto & odom_msg = odom_publisher_->msg_;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    // Position
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Velocity (in body frame as per REP-105)
    odom_msg.twist.twist.linear.x = vx_body;
    odom_msg.twist.twist.linear.y = vy_body;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega_body;

    // Covariance
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.05;

    odom_msg.twist.covariance.fill(0.0);
    odom_msg.twist.covariance[0] = 0.02;
    odom_msg.twist.covariance[7] = 0.02;
    odom_msg.twist.covariance[35] = 0.1;

    odom_publisher_->unlockAndPublish();
  }

  // Publish TF transform
  if (enable_odom_tf_ && tf_publisher_ && tf_publisher_->trylock())
  {
    auto & tf_msg = tf_publisher_->msg_;
    tf_msg.transforms.resize(1);
    auto & transform = tf_msg.transforms[0];

    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;

    transform.transform.translation.x = odom_x_;
    transform.transform.translation.y = odom_y_;
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_theta_);
    transform.transform.rotation = tf2::toMsg(q);

    tf_publisher_->unlockAndPublish();
  }
}

}  // namespace omni_base_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  omni_base_controller::OmniBaseController,
  controller_interface::ControllerInterface)