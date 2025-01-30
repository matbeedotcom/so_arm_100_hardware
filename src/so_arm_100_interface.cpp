#include "so_arm_100_hardware/so_arm_100_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace so_arm_100_controller
{
SOARM100Interface::SOARM100Interface() 
{
}

SOARM100Interface::~SOARM100Interface()
{
    if (SerialPort != -1)
    {
        close(SerialPort);
    }
}

CallbackReturn SOARM100Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  // Get parameters from hardware info
  use_serial_ = hardware_info.hardware_parameters.count("use_serial") ?
    (hardware_info.hardware_parameters.at("use_serial") == "true") : false;
  
  serial_port_ = hardware_info.hardware_parameters.count("serial_port") ?
    hardware_info.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";
  
  serial_baudrate_ = hardware_info.hardware_parameters.count("serial_baudrate") ?
    std::stoi(hardware_info.hardware_parameters.at("serial_baudrate")) : 1000000;

  size_t num_joints = info_.joints.size();
  position_commands_.resize(num_joints, 0.0);
  position_states_.resize(num_joints, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SOARM100Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SOARM100Interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn SOARM100Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Activating so_arm_100 hardware interface...");

  // Initialize joint states
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    position_states_[i] = 0.0;
    position_commands_[i] = 0.0;
  }

  // Initialize serial communication if enabled
  if (use_serial_) {
    SerialPort = open(serial_port_.c_str(), O_RDWR);
    if (SerialPort == -1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                   "Error opening serial port %s: %s", 
                   serial_port_.c_str(), strerror(errno));
      return CallbackReturn::ERROR;
    }

    if (!ConfigureSerialPort()) {
      RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                   "Failed to configure serial port");
      close(SerialPort);
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Serial communication initialized on %s", serial_port_.c_str());
  }

  // Initialize ROS communication
  node_ = rclcpp::Node::make_shared("so_arm_100_driver");

  feedback_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "feedback", 10, std::bind(&SOARM100Interface::feedback_callback, this, std::placeholders::_1));

  command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("command", 10);

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SOARM100Interface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool SOARM100Interface::ConfigureSerialPort()
{
  if (tcgetattr(SerialPort, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                 "Error getting serial port attributes: %s", strerror(errno));
    return false;
  }

  tty.c_cflag &= ~PARENB;        // No parity
  tty.c_cflag &= ~CSTOPB;        // 1 stop bit
  tty.c_cflag &= ~CSIZE;         // Clear size bits
  tty.c_cflag |= CS8;            // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

  tty.c_lflag &= ~ICANON;        // Non-canonical mode
  tty.c_lflag &= ~ECHO;          // Disable echo
  tty.c_lflag &= ~ECHOE;         // Disable erasure
  tty.c_lflag &= ~ECHONL;        // Disable new-line echo
  tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                   // Turn off software flow control
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes

  tty.c_oflag &= ~OPOST;         // Prevent special interpretation of output bytes
  tty.c_oflag &= ~ONLCR;         // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;          // Wait for up to 1s (10 deciseconds)
  tty.c_cc[VMIN] = 0;            // No minimum number of characters

  // Set baud rate
  speed_t baud = B1000000;  // Default
  switch(serial_baudrate_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 1000000: baud = B1000000; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                  "Unsupported baud rate %d, using 115200", serial_baudrate_);
  }

  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  if (tcsetattr(SerialPort, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                 "Error setting serial port attributes: %s", strerror(errno));
    return false;
  }

  return true;
}

void SOARM100Interface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Feedback received!");
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_feedback_msg_ = msg;
}

hardware_interface::return_type SOARM100Interface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (use_serial_) {
        unsigned char buf[256];
        int len = 0;
        
        // Format command packet
        buf[len++] = 0xFF;  // Start byte
        buf[len++] = 0xFF;  // Start byte
        
        // Add joint positions
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            // Convert position to servo value and add to buffer
            // Add your servo value conversion here
            buf[len++] = static_cast<unsigned char>(position_commands_[i]);
        }
        
        // Write to serial port
        if (WriteToSerial(buf, len) != len) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Failed to write to serial port");
        }
    }

    // Always publish to ROS topic for monitoring
    if (command_publisher_) {
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = node_->now();
        
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            cmd_msg.name.push_back(info_.joints[i].name);
            cmd_msg.position.push_back(position_commands_[i]);
        }
        
        command_publisher_->publish(cmd_msg);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SOARM100Interface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (use_serial_) {
        unsigned char buf[256];
        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Reading from serial port");
        int n = ReadSerial(buf, sizeof(buf));
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Read %d bytes from serial port", n);
        if (n > 0) {
            // Parse feedback packet
            // Example parsing (adjust according to your protocol):
            if (buf[0] == 0xFF && buf[1] == 0xFF) {
                for (size_t i = 0; i < info_.joints.size(); ++i) {
                    if (i + 2 < static_cast<size_t>(n)) {
                        // Convert servo value to position
                        // Add your position conversion here
                        position_states_[i] = static_cast<double>(buf[i + 2]);
                    }
                }
            }
        }
    }
    else {
        // Use ROS topic feedback
        sensor_msgs::msg::JointState::SharedPtr feedback_copy;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            feedback_copy = last_feedback_msg_;
        }

        if (feedback_copy) {
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                auto it = std::find(feedback_copy->name.begin(), feedback_copy->name.end(), info_.joints[i].name);
                if (it != feedback_copy->name.end()) {
                    size_t idx = std::distance(feedback_copy->name.begin(), it);
                    if (idx < feedback_copy->position.size()) {
                        position_states_[i] = feedback_copy->position[idx];
                    }
                }
            }
        }
    }

    return hardware_interface::return_type::OK;
}

int SOARM100Interface::WriteToSerial(const unsigned char* buf, int nBytes)
{
    if (SerialPort == -1)
    {
        // Simulation mode
        return nBytes; 
    }

    return ::write(SerialPort, const_cast<unsigned char*>(buf), nBytes);
}

int SOARM100Interface::ReadSerial(unsigned char* buf, int nBytes)
{
    if (SerialPort == -1)
    {
        // Simulation mode
        std::fill(buf, buf + nBytes, 0);
        return nBytes;
    }

    auto t_start = std::chrono::high_resolution_clock::now();
    int n = 0;
    while(n < nBytes)
    {
        int ret = ::read(SerialPort, &buf[n], 1);
        if(ret < 0)
        {
            return ret;
        }

        n += ret;
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 10000)  // 10 second timeout
        {
            break;
        }
    }
    return n;
}

}  // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface, hardware_interface::SystemInterface)
