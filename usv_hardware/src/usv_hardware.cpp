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



#include "usv_hardware.hpp"

/*
 * \brief 初始化父类和基节点
 */
hardware_interface::CallbackReturn USVHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    // 首先执行基类的初始化，确保成功
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
        return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = std::make_shared<rclcpp::Node>("USVHardware");
    RCLCPP_DEBUG(this->node_->get_logger(), "init成功！");
    // 此处应当有数量、类型之类的检查，但是由于是私用包，省略
    return hardware_interface::CallbackReturn::SUCCESS;
}

/*
 * \brief 启动时配置节点，创建realtime_publisher
 */
hardware_interface::CallbackReturn USVHardware::on_configure(const rclcpp_lifecycle::State &)
{
    subscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "state",
        rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) -> void{
            RCLCPP_DEBUG(this->node_->get_logger(), "接收到state！");
            msg_ptr_box.set(std::move(msg));  // 在不违反实时约束的情况下，在实时和非实时环境中安全地交换数据
        }
    );
    _ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("command", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

/*
 * \brief 每一次激活时配置其信息
 * \note 初始化那些只用在活动时才使用的信息
 */
hardware_interface::CallbackReturn USVHardware::on_activate(const rclcpp_lifecycle::State&)
{
    std::shared_ptr<std_msgs::msg::Float64MultiArray> empty_initarray = std::make_shared<std_msgs::msg::Float64MultiArray>();
    empty_initarray->data.emplace_back(std::nan("1"));  // 初始化，防止未定义行为
    msg_ptr_box.set(empty_initarray);

    RCLCPP_DEBUG(this->node_->get_logger(), "激活成功！");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn USVHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

/*
 * \brief 暴露Command Interface
 */
std::vector<hardware_interface::CommandInterface> USVHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // CommandInterface中是两个马达的转速
    for (unsigned char i{ 0 }; i < 2; i++)  
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "motor_speed", &motor_speeds_[i]
            )
        );
    }

    return command_interfaces;
}

/*
 * \brief 暴露State Interface
 */
std::vector<hardware_interface::StateInterface> USVHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // MPU6050反馈的速度和角速度
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            "MPU6050", "angular_velocity_z", &imu_msg_.angular_velocity_z
        )
    );

    state_interfaces.emplace_back(hardware_interface::StateInterface(
            "MPU6050", "linear_acceleration_x", &imu_msg_.linear_acceleration_x
        )
    );

    state_interfaces.emplace_back(hardware_interface::StateInterface(
            "MPU6050", "linear_acceleration_y", &imu_msg_.linear_acceleration_y
        )
    );

    // HMC5883L反馈的磁力大小
    state_interfaces.emplace_back(hardware_interface::StateInterface(
            "HMC5883L", "angle_z", &odometry_msg_.angle_z
        )
    );

    return state_interfaces;
}

/*
 * \brief 以一个频率从硬件读取信息，并更新内部状态
 */
hardware_interface::return_type USVHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    std::shared_ptr<std_msgs::msg::Float64MultiArray> msg_buffer_ptr;
    msg_ptr_box.get(msg_buffer_ptr);
    rclcpp::spin_some(this->node_);  // 使用spin_some方法可以不堵塞read还有write方法的执行，它只处理现在在事件队列中相关的回调

    imu_msg_.angular_velocity_z = msg_buffer_ptr->data[0];
    imu_msg_.linear_acceleration_x = msg_buffer_ptr->data[1];
    imu_msg_.linear_acceleration_y = msg_buffer_ptr->data[2];

    odometry_msg_.angle_z = msg_buffer_ptr->data[3];

    teleop_msg_.speed = static_cast<char>(msg_buffer_ptr->data[4]);
    teleop_msg_.angular_speed = static_cast<char>(msg_buffer_ptr->data[5]);
    teleop_msg_.is_teleoperated = msg_buffer_ptr->data[6] > 0;

    return hardware_interface::return_type::OK;
}

/*
 * \brief 以一个频率发送数据到硬件
 */
hardware_interface::return_type USVHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if(realtime_publisher_->trylock())
    {
        realtime_publisher_->msg_.data = motor_speeds_;
        realtime_publisher_->unlockAndPublish();
    }
    RCLCPP_DEBUG(this->node_->get_logger(), "motor_speed:%lf, %lf", motor_speeds_[0], motor_speeds_[1]);

    return hardware_interface::return_type::OK;  // motor_speeds是惯性数据，发送失败也可以返回OK
}

/*
 * \brief 注册插件
 */
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  USVHardware, hardware_interface::SystemInterface)