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



#include "controller_interface/controller_interface.hpp"
#include "usv_controller_parameter.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_box.h"
#include "boost/lockfree/queue.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

struct Handles{
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> motor_speed_handles; // command
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> imu_handles;  // state
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> magnetic_field_handles;  // state
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> teleop_handles;  // state  
};

// 与SystemInterface不同的是，ControllerInterface自带基节点，使用get_node()方法获取
// 使用boost无锁队列而不是realtime_tools提供的realtime_box来支持消息队列，实现高实时性
// 这里发布磁力计和IMU的信息使用robot_localization包发布\odom话题
class USVController : public controller_interface::ControllerInterface
{
public:
    USVController() : controller_interface::ControllerInterface() { }
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

    controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    std::shared_ptr<usv_controller::ParamListener> param_listener_;
    usv_controller::Params params_;
    boost::lockfree::queue<geometry_msgs::msg::Twist> received_twist_msg_ptr_ { 3 };  // 接收到的Twist信息队列，最多接收3个
    realtime_tools::RealtimeBox<nav_msgs::msg::Odometry::SharedPtr> odometry_msg_ptr_box_; 

    Handles handles_;  // HardwareInterface句柄
    
    std::vector<std::string> command_interface_types_{"left_motor/motor_speed", "right_motor/motor_speed"};
    std::vector<std::string> state_interface_types_{
        "MPU6050/angular_velocity_z",
        "MPU6050/linear_acceleration_x",
        "MPU6050/linear_acceleration_y",

        "HMC5883L/angle_z",

        "FS-IA6B/speed",
        "FS-IA6B/angular_speed",
        "FS-IA6B/is_teleoperated",
    };

    // 发送信息的缓存
    sensor_msgs::msg::Imu imu_msgs_;
    nav_msgs::msg::Odometry odometry_msgs_;
    geometry_msgs::msg::TransformStamped transform_stamped_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;  // 指令的订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;  // odom信息的订阅者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher__;
    realtime_tools::RealtimePublisherSharedPtr<sensor_msgs::msg::Imu> imu_realtime_publisher_;  // IMU实时发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr angular_publisher__;
    realtime_tools::RealtimePublisherSharedPtr<nav_msgs::msg::Odometry> angular_realtime_publisher_;  // MAG实时发布者
};