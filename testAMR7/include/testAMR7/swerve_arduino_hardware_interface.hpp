#ifndef SWERVE_ARDUINO_HARDWARE_INTERFACE_HPP
#define SWERVE_ARDUINO_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>
#include <termios.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sys/select.h>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace swerve_arduino_controller
{

class SwerveArduinoInterface : public hardware_interface::SystemInterface
{
public:
    SwerveArduinoInterface();
    virtual ~SwerveArduinoInterface();

    // Lifecycle methods
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // Interface export methods
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Read/Write methods
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Serial communication helpers
    bool setupSerialCommunication();
    void configureSerialPort();
    void waitForArduinoInitialization();
    bool readArduinoSensorData();
    bool processArduinoData(unsigned char* buffer, ssize_t bytes_read);
    int WriteToSerial(const unsigned char* buf, int nBytes);
    int ReadSerial(unsigned char* buf, int nBytes);
    std::string serial_buffer_;  // ← ADD THIS LINE for accumulating partial serial data

    // Serial port variables
    int SerialPort = -1;
    struct termios tty;

    // ROS2 node and publishers
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;

    // Command and state storage
    std::vector<double> velocity_commands_;
    std::vector<double> position_commands_;  // ← ADDED FOR STEERING POSITION CONTROL
    std::vector<double> velocity_states_;
    std::vector<double> position_states_;
};

}  // namespace swerve_arduino_controller

#endif  // SWERVE_ARDUINO_HARDWARE_INTERFACE_HPP