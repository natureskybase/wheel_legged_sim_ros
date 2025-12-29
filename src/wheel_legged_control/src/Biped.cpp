using namespace std;
#include <eigen3/Eigen/Dense>
#include "Biped.h"

Biped::Biped(const std::string &name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "biped运动控制节点启动");

    // tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    //     "/tf", 10,
    //     std::bind(&Biped::TFCallback, this, std::placeholders::_1));

    // rb_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //     "/cheat/velocity_world", 10,
    //     std::bind(&Biped::rbVelCallback, this, std::placeholders::_1));
};
