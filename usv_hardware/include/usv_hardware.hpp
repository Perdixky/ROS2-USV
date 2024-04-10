/***********************************************************************
 * Author: Perdixky
 * 
 * This file is part of a software project that is distributed under 
 * the GNU General Public License v3.0 (GPL-3.0). The GPL is a free, 
 * copyleft license for software and other kinds of works, offering the 
 * users the freedom to use, study, share, and modify the software.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 ***********************************************************************/



#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_box.h"

namespace simple
{
    struct Imu{
        double angular_velocity_z;
        double linear_acceleration_x;
        double linear_acceleration_y;
    };

    struct Odometry{
        double angle_z;
    };

    struct Teleop{
        bool is_teleoperated;
        unsigned char speed;
        unsigned char angular_speed;
    }
}

class USVHardware : public hardware_interface::SystemInterface  // 无node成员
{
public:

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
    // 下面三个基本数据都有初始化，数据传递时按照此定义的顺序进行
    simple::Imu imu_msg_;  // sensor_msgs::msg::Imu类型的数据，存储IMU的存储
    simple::Odometry odometry_msg_;  // sensor_msgs::msg::MagneticFeild类型数据，存储磁力计数据
    simple::Teleop teleop_msg_;
    std::vector<double> motor_speeds_{ std::nan("1") };   // 电机的转动速度（相对值，-100~100）


    realtime_tools::RealtimeBox<std_msgs::msg::Float64MultiArray::SharedPtr> msg_ptr_box;  // 无锁，双重缓冲

    rclcpp::Node::SharedPtr node_;  // 硬件抽象层的基节点

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _;  // 传入realtime_publisher
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Float64MultiArray> realtime_publisher_;  // 具有独立线程的高实时性publisher
}; 
