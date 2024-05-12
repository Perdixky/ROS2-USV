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



#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "realtime_tools/realtime_box.h"
#include "bit"
#include "std_msgs/msg/u_int8_multi_array.hpp"
serial::Serial ros_serial;
union Double_Byte
{
    double value;
    unsigned char bytes[8];
};


class SerialPublisher : public rclcpp::Node
{
public:
    SerialPublisher() : rclcpp::Node("serial_publisher")
    {
        sensor_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("state", rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(std::chrono::milliseconds(120), [this](){ this->timer_callback(); });
        subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("command", rclcpp::SystemDefaultsQoS(),
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msgs){
            this->subscription_callback(msgs);
        });
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensor_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
    realtime_tools::RealtimeBox<std::string> box;

    /*
     * \brief 以一定频率（120ms）从串口的缓冲中读取数据
     * \note 接收的数据中，angular是float类型，其余两个是乘以量程除以35535，量程以下位机配置传感器时的为准
     */
    void timer_callback()
    {
        std_msgs::msg::Float64MultiArray msgs;
        std::string data = ros_serial.read(ros_serial.available());
        // RCLCPP_INFO(this->get_logger(), "接收到的数据：%s", data.c_str());
        ros_serial.flushInput();
        auto pos = data.find('\n');
        if (pos + 12 <= data.size() && pos != std::string::npos)  // 检查是否没有\n或者是否pos后没有12个数据
        {
            // 从data中提取数据并进行转换
            short origin_angular_speed_z = *reinterpret_cast<const short*>(data.data() + 1);
            short origin_accelerate_x = *reinterpret_cast<const short*>(data.data() + 3);
            short origin_accelerate_y = *reinterpret_cast<const short*>(data.data() + 5);
            float angle = *reinterpret_cast<const float*>(data.data() + 7);
            // RCLCPP_INFO(this->get_logger(), "%d %d %d", static_cast<int>(data[11]), static_cast<int>(data[12]), static_cast<int>(data[13]));

            // 将提取的数据转换为目标格式并存储
            msgs.data.emplace_back(static_cast<double>(origin_angular_speed_z) * 1.3316e-5);  // angular_speed_z，50°/65535->rad/s
            msgs.data.emplace_back(static_cast<double>(origin_accelerate_x) * 5.515688757563e-4);  // accelerate_x，2g->m/s²
            msgs.data.emplace_back(static_cast<double>(origin_accelerate_y) * 5.515688757563e-4);  // accelerate_y，2g->m/s²
            msgs.data.emplace_back(static_cast<double>(angle));  // angle
            msgs.data.emplace_back(static_cast<double>(data[11]));  // teleop_speed
            msgs.data.emplace_back(static_cast<double>(data[12]));  // teleop_angular_speed
            msgs.data.emplace_back(static_cast<double>(data[13]));  // is_teleoperated
            sensor_publisher_->publish(msgs);
        }
    }

    /*
     * \brief 发布速度数据，速度的最大值为1m/s，乘以100以unsigned char类型串口发送
     */
    void subscription_callback(std_msgs::msg::Float64MultiArray::SharedPtr msgs)
    {
        std::string buffer{ };
        buffer += static_cast<unsigned char>(msgs->data[0]);
        buffer += static_cast<unsigned char>(msgs->data[1]);
        ros_serial.write(buffer);
    }
};

auto main(int argc, char** argv) -> int
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPublisher>();
    ros_serial.setPort("/dev/ttyS3");  // 取决于具体硬件
    ros_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ros_serial.setTimeout(to);
    try{
        ros_serial.open();
    }catch(serial::IOException &e){
        RCLCPP_FATAL(node->get_logger(), "串口未成功打开：%s", e.what());
        return -1;
    }
    if(ros_serial.isOpen())
        RCLCPP_INFO(node->get_logger(), "串口成功打开！");
    rclcpp::spin(node);
    rclcpp::shutdown();
    ros_serial.close();
    return 0;
}