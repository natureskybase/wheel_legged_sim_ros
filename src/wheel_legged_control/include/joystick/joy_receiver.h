#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mutex>

class JoyReceiver : public rclcpp::Node
{
public:
    JoyReceiver();
    // 单例访问接口
    static std::shared_ptr<JoyReceiver> getInstance();

    // 公共变量：便于外部直接访问遥感数据（你可根据自己的遥控器定制字段）
    float lx;                 // 左摇杆 左右
    float ly;                 // 左摇杆 上下
    float rx;                 // 右摇杆 左右
    float ry;                 // 右摇杆 上下
    std::array<int,11> buttons; // 所有按钮状态

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::mutex data_mutex_;
};
