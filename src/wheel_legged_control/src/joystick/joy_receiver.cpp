#include "joy_receiver.h"

std::shared_ptr<JoyReceiver> JoyReceiver::getInstance()
{
    static std::shared_ptr<JoyReceiver> instance = std::make_shared<JoyReceiver>();
    return instance;
}

JoyReceiver::JoyReceiver() : Node("joy_receiver")
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&JoyReceiver::joyCallback, this, std::placeholders::_1));

    // 初始化成员变量
    lx = 0.0f;
    ly = 0.0f;
    rx = 0.0f;
    ry = 0.0f;
}

void JoyReceiver::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 安全地更新成员变量（注意 axes 长度检查）
    if (msg->axes.size() >= 4)
    {
        lx = msg->axes[0];
        ly = msg->axes[1];
        rx = msg->axes[3];
        ry = msg->axes[4];
    }

    for (int i = 0; i < 11; i++)
    {
        buttons[i] = msg->buttons[i];
    }
}
