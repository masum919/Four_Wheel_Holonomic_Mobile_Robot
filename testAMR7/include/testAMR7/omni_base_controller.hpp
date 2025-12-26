#ifndef OMNI_BASE_CONTROLLER__OMNI_BASE_CONTROLLER_HPP_
#define OMNI_BASE_CONTROLLER__OMNI_BASE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace omni_base_controller
{

class OmniBaseController : public controller_interface::ControllerInterface
{
public:
  OmniBaseController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:

  double last_vx_body_{0.0};
  double last_vy_body_{0.0};
  double last_omega_body_{0.0};

protected:
  std::array<double, 4> prev_wheel_velocities_;  // For velocity smoothing
  // Wheel module structure
  struct WheelModule
  {
    double x;  // X position relative to robot center
    double y;  // Y position relative to robot center
    double wheel_velocity;  // Current wheel velocity command
    double steer_angle;     // Current steering angle command
    double steer_angle_prev; // Previous steering angle for optimization
  };

  // Robot parameters
  double wheel_radius_;
  double wheel_base_;   // Distance between front and rear axles
  double wheel_track_;  // Distance between left and right wheels
  
  // Velocity limits
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_wheel_velocity_;
  double max_steer_velocity_;
  
  // Control parameters
  double cmd_vel_timeout_;
  double steer_angle_tolerance_;
  bool use_shortest_path_;
  bool enable_odom_tf_;
  bool enable_odometry_;  // Master switch for odometry
  double publish_rate_;
  
  // Steering priority parameters
  double steering_threshold_;  // Angle difference threshold for steering priority
  double velocity_scale_factor_;  // Scale factor for wheel velocity during steering; how much wheels should roll during steering; stop -> reorient -> go
  
  // Frame IDs
  std::string odom_frame_id_;
  std::string base_frame_id_;
  
  // Joint names
  std::vector<std::string> wheel_joint_names_;
  std::vector<std::string> steer_joint_names_;
  
  // Wheel modules (BR, FR, FL, BL)
  std::array<WheelModule, 4> wheel_modules_;
  
  // Odometry
  double odom_x_;
  double odom_y_;
  double odom_theta_;
  rclcpp::Time last_odom_time_;
  
  // Command subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> received_velocity_msg_ptr_;
  rclcpp::Time last_cmd_time_;
  
  // Odometry publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_publisher_;
  
  // Methods
  void cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  
  void computeInverseKinematics(
    double vx, double vy, double omega,
    std::array<double, 4> & wheel_velocities,
    std::array<double, 4> & steer_angles);
  
  void optimizeSteeringAngles(
    std::array<double, 4> & wheel_velocities,
    std::array<double, 4> & steer_angles);
  
  double normalizeAngle(double angle);
  
  void updateOdometry(const rclcpp::Time & time, const rclcpp::Duration & period);
  
  void publishOdometry(const rclcpp::Time & time);
  
  // Check if steering angles need adjustment
  bool needsSteeringAdjustment(const std::array<double, 4> & target_angles);
};

}  // namespace omni_base_controller

#endif  // OMNI_BASE_CONTROLLER__OMNI_BASE_CONTROLLER_HPP_