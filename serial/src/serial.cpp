#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "realtime_tools/realtime_box.h"

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
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("state", rclcpp::SystemDefaultsQoS());
        timer_ = this->create_wall_timer(std::chrono::milliseconds(80), [this](){ this->timer_callback(); });
        subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("command", rclcpp::SystemDefaultsQoS(),
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msgs){
            this->subscription_callback(msgs);
        });
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
    realtime_tools::RealtimeBox<std::string> box;

    /*
     * \brief 以一定频率（80ms）从串口的缓冲中读取数据
     * \note 接收的数据中，angular是char类型，其余两个是乘以量程除以35535，量程以下位机配置传感器时的为准
     */
    void timer_callback()
    {
        std_msgs::msg::Float64MultiArray msgs;
        std::string data = ros_serial.read(6);
        ros_serial.flushInput();
        auto pos = data.find('\n');
        if (pos + 3 >= data.size() && pos == std::string::npos)
        {
            RCLCPP_ERROR(this->get_logger(), "接收的数据有问题！");
            std::cout << data;
        }
        msgs.data.emplace_back(static_cast<double>(data[pos + 1]));
        msgs.data.emplace_back(static_cast<double>(data[pos + 2]) * 5.515688757563e-4);
        msgs.data.emplace_back(static_cast<double>(data[pos + 3]) * 0.00703532);
        publisher_->publish(msgs);
    }

    /*
     * \brief 发布速度数据，速度的最大值为2.55m/s，乘以100以unsigned char类型串口发送
     */
    void subscription_callback(std_msgs::msg::Float64MultiArray::SharedPtr msgs)
    {
        std::string buffer{ };
        buffer += static_cast<unsigned char>((*msgs->data.begin()) * 100);
        buffer += static_cast<unsigned char>((*msgs->data.end()) * 100);
        ros_serial.write(buffer);
    }
};

auto main(int argc, char** argv) -> int
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPublisher>();
    ros_serial.setPort("/dev/ttyS3");
    ros_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ros_serial.setTimeout(to);
    return 0;
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