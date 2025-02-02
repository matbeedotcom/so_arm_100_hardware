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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace so_arm_100_controller
{
SOARM100Interface::SOARM100Interface() 
{
}

SOARM100Interface::~SOARM100Interface()
{
    if (use_serial_) {
        st3215_.end();
    }
}

CallbackReturn SOARM100Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

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

    if (use_serial_) {
        if(!st3215_.begin(serial_baudrate_, serial_port_.c_str())) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Failed to initialize motors");
            return CallbackReturn::ERROR;
        }

        // Initialize each servo
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            // First ping the servo
            if (st3215_.Ping(servo_id) == -1) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "No response from servo %d during initialization", servo_id);
                return CallbackReturn::ERROR;
            }
            
            // Set to position control mode
            if (!st3215_.Mode(servo_id, 0)) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "Failed to set mode for servo %d", servo_id);
                return CallbackReturn::ERROR;
            }

            // Read initial position and set command to match
            if (st3215_.FeedBack(servo_id) != -1) {
                int pos = st3215_.ReadPos(servo_id);
                position_states_[i] = M_PI - (pos * 2 * M_PI / 4096.0);
                position_commands_[i] = position_states_[i];  // Set command to match current position
                RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                           "Servo %d initialized at position %d", servo_id, pos);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Serial communication initialized on %s", serial_port_.c_str());
    }

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
    
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            st3215_.EnableTorque(servo_id, 0);
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

void SOARM100Interface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_msg_ = msg;
}

hardware_interface::return_type SOARM100Interface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            int joint_pos_cmd = static_cast<int>(round((M_PI - position_commands_[i]) * 4096.0 / (2 * M_PI))); // Convert to encoder ticks
            
            if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, 4500, 255)) {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                           "Failed to write position to servo %d", servo_id);
            }
        }
        st3215_.RegWriteAction();
    }

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
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            // Add small delay between reads
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            if (st3215_.FeedBack(servo_id) != -1) {
                // Read all available data
                double position = M_PI - st3215_.ReadPos(servo_id) * 2 * M_PI / 4096.0;
                double speed = -1 * st3215_.ReadSpeed(servo_id) * 2 * M_PI / 4096.0;
                double pwm = -1 * st3215_.ReadLoad(servo_id) / 10.0;
                int move = st3215_.ReadMove(servo_id);
                double temperature = st3215_.ReadTemper(servo_id);
                double voltage = st3215_.ReadVoltage(servo_id) / 10;
                double current = st3215_.ReadCurrent(servo_id) * 6.5 / 1000;

                position_states_[i] = position;

                RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"), 
                            "Servo %d: pos=%.2f speed=%.2f pwm=%.2f temp=%.1f V=%.1f I=%.3f", 
                            servo_id, position, speed, pwm, temperature, voltage, current);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                           "Failed to read feedback from servo %d", servo_id);
            }
        }
    }
    else {
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

}  // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface, hardware_interface::SystemInterface)
