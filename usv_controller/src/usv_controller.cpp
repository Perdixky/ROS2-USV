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

#include "usv_controller.hpp"

controller_interface::InterfaceConfiguration USVController::command_interface_configuration() const
{
    return {controller_interface::interface_configuration_type::INDIVIDUAL, command_interface_types_};
}

controller_interface::InterfaceConfiguration USVController::state_interface_configuration() const
{
    return {controller_interface::interface_configuration_type::INDIVIDUAL, state_interface_types_};
}
/*
 * \brief 初始化ParamListener
 */
controller_interface::CallbackReturn USVController::on_init()
{
    RCLCPP_DEBUG(get_node()->get_logger(), "启动节点……");
    try{
        param_listener_ = std::make_shared<usv_controller::ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }catch(const std::exception& e){
        RCLCPP_FATAL(get_node()->get_logger(), "节点监听器启动失败：%s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

/*
 * \brief 配置节点
 */
controller_interface::CallbackReturn USVController::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_DEBUG(get_node()->get_logger(), "配置节点……");

    //更新参数
    if(param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        RCLCPP_DEBUG(get_node()->get_logger(), "steering_sensitivity: %ld\nmotor_speed_ratio: %lf\npid: %lf, %lf, %lf", 
        params_.steering_sensitivity, params_.motor_speed_ratio, *params_.pid.begin(), *(params_.pid.begin() + 1), *(params_.pid.begin() + 2));
        RCLCPP_INFO(get_node()->get_logger(), "更新参数成功");
    }

    // 初始化command_subscriber
    command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist> ("/cmd_vel", rclcpp::SystemDefaultsQoS(), 
        [this](const geometry_msgs::msg::Twist::SharedPtr command){
            received_twist_msg_ptr_.push(*command);
        }
    );

    // 初始化odometry_subscriber
    odometry_subscriber_ = get_node()->create_subscription<nav_msgs::msg::Odometry> ("/odometry/filtered", rclcpp::SystemDefaultsQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr odom){
            odometry_msg_ptr_box_.set(odom);
        }
    );

    // 初始化imu_publisher
    imu_publisher__ = get_node()->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SystemDefaultsQoS());
    imu_realtime_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(imu_publisher__);
    angular_publisher__ = get_node()->create_publisher<nav_msgs::msg::Odometry>("angle", rclcpp::SystemDefaultsQoS());
    angular_realtime_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(angular_publisher__);

    // 初始化需要发布的信息
    imu_msgs_.header.frame_id = "sensor_link";
    imu_msgs_.angular_velocity_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // 不提供数据时，协方差矩阵第一个元素设为-1；协方差矩阵未知时，应全部设为0
    imu_msgs_.linear_acceleration_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    imu_msgs_.orientation_covariance = { -1, 0, 0, 0, 0, 0, 0, 0, 0 };

    odometry_msgs_.header.frame_id = "world";  // 磁力计数据相对于世界
    odometry_msgs_.pose.covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    odometry_msgs_.twist.covariance = { -1, 0, 0, 0, 0, 0, 0, 0, 0 };

    return controller_interface::CallbackReturn::SUCCESS;
}

/*
 * \brief 激活节点
 */
controller_interface::CallbackReturn USVController::on_activate(const rclcpp_lifecycle::State &)
{
    // 获取Interface句柄
    for (auto& state_interface : state_interfaces_)
    {
        if (state_interface.get_prefix_name() == "MPU6050")
            handles_.imu_handles.emplace_back(std::ref(state_interface));
        else if(state_interface.get_prefix_name() == "HMC5883L")
            handles_.magnetic_field_handles.emplace_back(std::ref(state_interface));
        else
            handles_.teleop_handles.emplace_back(std::ref(state_interface));
    }

    // 确定填充顺序
    auto& first_handle = command_interfaces_.begin()->get_prefix_name() == "left_motor" ? 
                        command_interfaces_[0] : command_interfaces_[1];
    auto& second_handle = &first_handle == &command_interfaces_[0] ? 
                      command_interfaces_[1] : command_interfaces_[0];
    // 填充handles_
    handles_.motor_speed_handles.emplace_back(std::ref(first_handle));
    handles_.motor_speed_handles.emplace_back(std::ref(second_handle));
    RCLCPP_DEBUG(get_node()->get_logger(), "controller激活成功");

    return controller_interface::CallbackReturn::SUCCESS;
}

/*
 * \brief 更新，包括更新CommandInterface和StateInterface并发布
 */
controller_interface::return_type USVController::update(const rclcpp::Time& time, const rclcpp::Duration &)
{
    params_ = param_listener_->get_params();

    // 发布数据给robot_localization，获取odom
    if (imu_realtime_publisher_->trylock())
    {
        imu_msgs_.header.stamp = time;
        imu_msgs_.angular_velocity.z = handles_.imu_handles[0].get().get_value();
        imu_msgs_.linear_acceleration.x = handles_.imu_handles[1].get().get_value();
        imu_msgs_.linear_acceleration.y = handles_.imu_handles[2].get().get_value();
        imu_realtime_publisher_->msg_ = imu_msgs_;
        imu_realtime_publisher_->unlockAndPublish();
    }
    if (angular_realtime_publisher_->trylock())
    {
        odometry_msgs_.header.stamp = time;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, handles_.magnetic_field_handles[0].get().get_value());  // 自动计算四元数
        odometry_msgs_.pose.pose.orientation.x = q.x();
        odometry_msgs_.pose.pose.orientation.y = q.y();
        odometry_msgs_.pose.pose.orientation.z = q.z();
        odometry_msgs_.pose.pose.orientation.w = q.w();

        angular_realtime_publisher_->msg_ = odometry_msgs_;
        angular_realtime_publisher_->unlockAndPublish();
    }

    // 使用pid算法调节获得StateInterface并发布命令
    geometry_msgs::msg::Twist twist_message;
    received_twist_msg_ptr_.pop(twist_message);
    std::shared_ptr<nav_msgs::msg::Odometry> odometry;
    odometry_msg_ptr_box_.get(odometry);

    if(handles_.teleop_handles[2].get().get_value() > 90)
    {
        auto sigmoid = [](int x) -> double{  // 将-50 ~ 150夹到0 ~ 100
            return 1 / (1 + std::exp(-0.02 * (x - 50)));
        };
        double speed_difference = params_.steering_sensitivity * handles_.teleop_handles[1].get().get_value();
        handles_.motor_speed_handles[0].get().set_value(sigmoid(handles_.teleop_handles[0].get().get_value() + speed_difference - 50));  // 0 ~ 100（不包含转弯），-50是因为speed_difference中间值是50
        handles_.motor_speed_handles[1].get().set_value(sigmoid(handles_.teleop_handles[0].get().get_value() + speed_difference - 50));
    }
    else{
        /*if (!odometry){
            return controller_interface::return_type::ERROR;
        }
        double speed_difference = *(params_.pid.begin()) * (twist_message.angular.z - odometry->twist.twist.angular.z);
        handles_.motor_speed_handles[0].get().set_value(twist_message.linear.x - speed_difference);
        handles_.motor_speed_handles[1].get().set_value(twist_message.linear.x + speed_difference);*/
        RCLCPP_INFO(this->get_node()->get_logger(), "请拉低SWD开关！！！");
    }
    return controller_interface::return_type::OK;
    
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  USVController, controller_interface::ControllerInterface)