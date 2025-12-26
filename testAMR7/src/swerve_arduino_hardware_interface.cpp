#include "testAMR7/swerve_arduino_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <sys/select.h>
#include <sys/ioctl.h>

namespace swerve_arduino_controller
{

// ============================================================================
// CONSTRUCTOR
// ============================================================================
SwerveArduinoInterface::SwerveArduinoInterface() 
{
}

// ============================================================================
// DESTRUCTOR
// ============================================================================
SwerveArduinoInterface::~SwerveArduinoInterface()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}

// ============================================================================
// LIFECYCLE METHODS
// ============================================================================

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

CallbackReturn SwerveArduinoInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Initialize vectors with correct size (8 joints: 4 steer + 4 wheel)
    velocity_commands_.resize(info_.joints.size(), 0.0);
    position_commands_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);

    RCLCPP_INFO(
        rclcpp::get_logger("SwerveArduinoInterface"),
        "Hardware interface initialized with %zu joints", 
        info_.joints.size()
    );

    return CallbackReturn::SUCCESS;
}

#pragma GCC diagnostic pop

CallbackReturn SwerveArduinoInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    node_ = std::make_shared<rclcpp::Node>("swerve_sensor_publisher_node");
    
    current_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motor_currents", 10);
    velocity_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motor_velocities", 10);
    position_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motor_positions", 10);

    try
    {
        if (!setupSerialCommunication())
        {
            RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"), 
                        "No Arduino connection - running in simulation mode");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                        "Arduino connected! Waiting for initialization...");
            waitForArduinoInitialization();
            RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                        "Arduino ready. Real hardware feedback enabled");
        }
    }
    catch(std::exception &e)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("swerve_arduino_interface"),
            "Error: %s. Running in simulation mode.", e.what()
        );
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SwerveArduinoInterface"), "Hardware configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveArduinoInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SwerveArduinoInterface"), "Activating hardware...");

    std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
    std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
    std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
    std::fill(position_states_.begin(), position_states_.end(), 0.0);

    RCLCPP_INFO(rclcpp::get_logger("SwerveArduinoInterface"),
                "Hardware active with %zu joints", info_.joints.size());
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn SwerveArduinoInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if(SerialPort == -1)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    // Send stop command
    std::string stop_cmd = "0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n";
    WriteToSerial(reinterpret_cast<const unsigned char*>(stop_cmd.c_str()), stop_cmd.length());
    
    tcflush(SerialPort, TCIFLUSH);
    close(SerialPort);
    
    RCLCPP_INFO(rclcpp::get_logger("SwerveArduinoInterface"), "Hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// INTERFACE EXPORT METHODS
// ============================================================================

std::vector<hardware_interface::StateInterface> SwerveArduinoInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SwerveArduinoInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Export POSITION commands for STEERING joints (first 4 joints)
    // URDF order: BR_steer, FR_steer, FL_steer, BL_steer
    for (size_t i = 0; i < 4; i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION,
            &position_commands_[i]));
    }

    // Export VELOCITY commands for ALL joints
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, 
            &velocity_commands_[i]));
    }

    RCLCPP_INFO(
        rclcpp::get_logger("SwerveArduinoInterface"),
        "Exported %zu command interfaces (4 position + 8 velocity)", 
        command_interfaces.size()
    );

    return command_interfaces;
}

// ============================================================================
// READ/WRITE METHODS
// ============================================================================

hardware_interface::return_type SwerveArduinoInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
    static int write_counter = 0;
    static auto last_log_time = std::chrono::steady_clock::now();
    write_counter++;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count();
    
    if (elapsed >= 1000) {
        double actual_hz = write_counter / (elapsed / 1000.0);
        RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                    "Write called %d times in last second (%.1f Hz)", 
                    write_counter, actual_hz);
        write_counter = 0;
        last_log_time = now;
    }
    
    if (SerialPort == -1)
    {
        static int sim_counter = 0;
        if (sim_counter++ % 100 == 0) {
            RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                        "SIMULATION MODE: Commands received but not sent to Arduino");
        }
        return hardware_interface::return_type::OK;
    }

    // Convert position commands to velocities for steering motors
    // Simple P-controller: vel = K * (target_pos - current_pos)
    try {
        const double K_steer = 5.0;  // Position gain
        
        // Calculate steering velocities from position error
        float steer_vel_br = static_cast<float>(K_steer * (position_commands_[0] - position_states_[0]));
        float steer_vel_fr = static_cast<float>(K_steer * (position_commands_[1] - position_states_[1]));
        float steer_vel_fl = static_cast<float>(K_steer * (position_commands_[2] - position_states_[2]));
        float steer_vel_bl = static_cast<float>(K_steer * (position_commands_[3] - position_states_[3]));
        
        // Wheel velocities come directly from velocity commands
        float wheel_vel_br = static_cast<float>(velocity_commands_[4]);
        float wheel_vel_fr = static_cast<float>(velocity_commands_[5]);
        float wheel_vel_fl = static_cast<float>(velocity_commands_[6]);
        float wheel_vel_bl = static_cast<float>(velocity_commands_[7]);

        // Arduino expects: BR_steer FR_steer FL_steer BL_steer BR_wheel FR_wheel FL_wheel BL_wheel
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << steer_vel_br << " " << steer_vel_fr << " " 
            << steer_vel_fl << " " << steer_vel_bl << " "
            << wheel_vel_br << " " << wheel_vel_fr << " " 
            << wheel_vel_fl << " " << wheel_vel_bl << "\n";

        std::string data = oss.str();
        int bytes_written = WriteToSerial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());

        // Verify write succeeded
        if (bytes_written != static_cast<int>(data.length())) {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("swerve_arduino_interface"),
                *node_->get_clock(), 1000,
                "Serial write incomplete: %d/%d bytes", 
                bytes_written, static_cast<int>(data.length())
            );
        }

        static int cmd_counter = 0;
        if (cmd_counter++ % 10 == 0) {
            RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                        "CMD #%d: Pos[%.2f %.2f %.2f %.2f] → Vel_S[%.2f %.2f %.2f %.2f] Vel_W[%.2f %.2f %.2f %.2f] (%dB)", 
                        cmd_counter,
                        position_commands_[0], position_commands_[1], position_commands_[2], position_commands_[3],
                        steer_vel_br, steer_vel_fr, steer_vel_fl, steer_vel_bl,
                        wheel_vel_br, wheel_vel_fr, wheel_vel_fl, wheel_vel_bl,
                        bytes_written);
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("swerve_arduino_interface"), "Write error: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SwerveArduinoInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    static int read_counter = 0;
    static auto last_read_log = std::chrono::steady_clock::now();
    read_counter++;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_log).count();
    
    if (elapsed >= 1000) {
        double actual_hz = read_counter / (elapsed / 1000.0);
        RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                    "Read called %d times in last second (%.1f Hz)", 
                    read_counter, actual_hz);
        read_counter = 0;
        last_read_log = now;
    }
    
    if (SerialPort == -1)
    {
        // SIMULATION MODE: Still publish at 100Hz
        std_msgs::msg::Float32MultiArray current_msg, vel_msg, pos_msg;
        for (int i = 0; i < 8; i++) {
            current_msg.data.push_back(0.0f);
            vel_msg.data.push_back(velocity_states_[i]);
            pos_msg.data.push_back(position_states_[i]);
        }
        current_publisher_->publish(current_msg);
        velocity_publisher_->publish(vel_msg);
        position_publisher_->publish(pos_msg);
        return hardware_interface::return_type::OK;
    }

    // Read ALL available data in one shot (non-blocking)
    unsigned char buffer[2048];  // Larger buffer for multiple messages
    std::memset(buffer, 0, sizeof(buffer));

    // Check if data available
    int bytes_available = 0;
    ioctl(SerialPort, FIONREAD, &bytes_available);
    
    if (bytes_available > 0) {
        // Read as much as possible
        ssize_t bytes = ::read(SerialPort, buffer, std::min(bytes_available, (int)sizeof(buffer) - 1));
        if (bytes > 0) {
            processArduinoData(buffer, bytes);
        }
    }
    
    static int no_data_count = 0;
    if (bytes_available == 0) {
        no_data_count++;
        if (no_data_count % 200 == 0) {
            RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"), 
                       "No data available for %d read cycles", no_data_count);
        }
    } else {
        no_data_count = 0;
    }

    return hardware_interface::return_type::OK;
}

// ============================================================================
// SERIAL COMMUNICATION HELPERS
// ============================================================================

bool SwerveArduinoInterface::setupSerialCommunication()
{
    std::string port = "/dev/ttyACM0";
    int baud_rate = 230400;

    auto port_param = info_.hardware_parameters.find("port");
    if (port_param != info_.hardware_parameters.end())
    {
        port = port_param->second;
    }

    auto baud_param = info_.hardware_parameters.find("baud_rate");
    if (baud_param != info_.hardware_parameters.end())
    {
        try {
            baud_rate = std::stoi(baud_param->second);
        } catch (...) {
            RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"), 
                        "Invalid baud_rate '%s', using 230400", baud_param->second.c_str());
            baud_rate = 230400;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                "Attempting to open serial port: %s @ %d baud", port.c_str(), baud_rate);
    
    SerialPort = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (SerialPort < 0)
    {
        RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"), 
                    "Cannot open %s: %s", port.c_str(), strerror(errno));
        return false;
    }

    int status;
    if (ioctl(SerialPort, TIOCMGET, &status) == 0)
    {
        status &= ~TIOCM_DTR;
        status &= ~TIOCM_RTS;
        ioctl(SerialPort, TIOCMSET, &status);
        RCLCPP_DEBUG(rclcpp::get_logger("swerve_arduino_interface"), 
                    "DTR and RTS lines cleared");
    }

    if (tcgetattr(SerialPort, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("swerve_arduino_interface"), 
                     "tcgetattr failed: %s", strerror(errno));
        close(SerialPort);
        return false;
    }

    // Configure port
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_oflag &= ~OCRNL;
    tty.c_oflag &= ~ONOCR;
    tty.c_oflag &= ~ONLRET;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    // Set baud rate
    speed_t speed;
    switch (baud_rate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 921600: speed = B921600; break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"),
                        "Unsupported baud %d, using 230400", baud_rate);
            speed = B230400;
            baud_rate = 230400;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    if (tcsetattr(SerialPort, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("swerve_arduino_interface"), 
                     "tcsetattr failed: %s", strerror(errno));
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                "Serial port %s opened successfully at %d baud", port.c_str(), baud_rate);

    // Wait for Arduino reset
    RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                "Waiting 2 seconds for Arduino reset...");
    usleep(2000000);
    tcflush(SerialPort, TCIOFLUSH);
    
    return true;
}

void SwerveArduinoInterface::configureSerialPort()
{
    // This function is now unused — config moved to setupSerialCommunication()
}

void SwerveArduinoInterface::waitForArduinoInitialization()
{
    RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                "Waiting for Arduino to send ready signal...");
    
    unsigned char buffer[256];
    int attempts = 0;
    const int max_attempts = 50;  // 5 seconds
    
    while (attempts < max_attempts)
    {
        usleep(100000);  // 100ms
        
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(SerialPort, &read_fds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        
        int ready = select(SerialPort + 1, &read_fds, NULL, NULL, &timeout);
        
        if (ready > 0)
        {
            ssize_t bytes = ::read(SerialPort, buffer, sizeof(buffer) - 1);
            if (bytes > 0)
            {
                buffer[bytes] = '\0';
                std::string msg(reinterpret_cast<char*>(buffer));
                
                // Accept any line starting with '#' as ready signal
                if (msg.find('#') != std::string::npos)
                {
                    RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                               "Arduino ready! Message: [%s]", msg.c_str());
                    usleep(200000);
                    tcflush(SerialPort, TCIOFLUSH);
                    return;
                }
            }
        }
        attempts++;
    }
    
    RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"), 
                "Arduino ready signal not detected after 5s - proceeding anyway");
}

bool SwerveArduinoInterface::readArduinoSensorData()
{
    if (SerialPort == -1) return false;

    unsigned char buffer[512];
    std::memset(buffer, 0, sizeof(buffer));

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(SerialPort, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms

    int ready = select(SerialPort + 1, &read_fds, NULL, NULL, &timeout);

    if (ready > 0)
    {
        ssize_t bytes = ::read(SerialPort, buffer, sizeof(buffer) - 1);
        if (bytes > 0)
        {
            return processArduinoData(buffer, bytes);
        }
    }
    
    return false;
}

// Add this to your class header as a private member:
// std::string serial_buffer_;  // Accumulates partial data

bool SwerveArduinoInterface::processArduinoData(unsigned char* buffer, ssize_t bytes_read)
{
    if (bytes_read <= 0) return false;

    // Append new data to accumulated buffer
    buffer[bytes_read] = '\0';
    serial_buffer_ += std::string(reinterpret_cast<char*>(buffer));

    // Process all complete lines in buffer
    bool processed_any = false;
    size_t nl_pos;
    
    while ((nl_pos = serial_buffer_.find('\n')) != std::string::npos) {
        // Extract one complete line
        std::string line = serial_buffer_.substr(0, nl_pos);
        serial_buffer_.erase(0, nl_pos + 1);  // Remove processed line
        
        // Remove \r if present
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        // Parse the line
        std::istringstream iss(line);
        float currents[8], velocities[8], positions[8];

        // Parse 24 floats: 8 currents, 8 velocities, 8 positions
        bool parse_ok = true;
        for (int i = 0; i < 8; i++) {
            if (!(iss >> currents[i])) { parse_ok = false; break; }
        }
        if (parse_ok) {
            for (int i = 0; i < 8; i++) {
                if (!(iss >> velocities[i])) { parse_ok = false; break; }
            }
        }
        if (parse_ok) {
            for (int i = 0; i < 8; i++) {
                if (!(iss >> positions[i])) { parse_ok = false; break; }
            }
        }

        if (!parse_ok) {
            static int parse_error_count = 0;
            if (++parse_error_count % 100 == 0) {
                RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"),
                           "Parse errors: %d (line: %s)", parse_error_count, line.substr(0, 50).c_str());
            }
            continue;
        }

        // Update states
        for (size_t i = 0; i < 8 && i < velocity_states_.size(); i++) {
            velocity_states_[i] = velocities[i];
            position_states_[i] = positions[i];
        }

        // Publish EVERY valid line we receive
        std_msgs::msg::Float32MultiArray current_msg, vel_msg, pos_msg;
        for (int i = 0; i < 8; i++) {
            current_msg.data.push_back(currents[i]);
            vel_msg.data.push_back(velocities[i]);
            pos_msg.data.push_back(positions[i]);
        }
        current_publisher_->publish(current_msg);
        velocity_publisher_->publish(vel_msg);
        position_publisher_->publish(pos_msg);

        processed_any = true;

        static int log_counter = 0;
        if (log_counter++ % 100 == 0) {
            RCLCPP_INFO(rclcpp::get_logger("swerve_arduino_interface"), 
                        "FEEDBACK #%d: S[BR:%.2f FR:%.2f FL:%.2f BL:%.2f] W[BR:%.2f FR:%.2f FL:%.2f BL:%.2f]", 
                        log_counter,
                        velocities[0], velocities[1], velocities[2], velocities[3],
                        velocities[4], velocities[5], velocities[6], velocities[7]);
        }
    }
    
    // Prevent buffer from growing too large
    if (serial_buffer_.size() > 1024) {
        RCLCPP_WARN(rclcpp::get_logger("swerve_arduino_interface"),
                   "Serial buffer overflow, clearing (%zu bytes)", serial_buffer_.size());
        serial_buffer_.clear();
    }

    return processed_any;
}

int SwerveArduinoInterface::WriteToSerial(const unsigned char* buf, int nBytes)
{
    int bytes_written = ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
    if (bytes_written != nBytes) {
        RCLCPP_ERROR(rclcpp::get_logger("swerve_arduino_interface"),
                     "Write incomplete: %d/%d bytes", bytes_written, nBytes);
    }
    return bytes_written;
}

int SwerveArduinoInterface::ReadSerial(unsigned char* buf, int nBytes)
{
    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0) return ret;
        n += ret;
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed > 10000) break;
    }
    return n;
}

}  // namespace swerve_arduino_controller

PLUGINLIB_EXPORT_CLASS(swerve_arduino_controller::SwerveArduinoInterface, hardware_interface::SystemInterface)