#ifndef LIBBIOMIMETICS_BIPED_H
#define LIBBIOMIMETICS_BIPED_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <cppType.h>

/**
 * @brief Biped类，表示一个双足机器人
 * @details 该类继承自rclcpp::Node，封装了双足机器人的以下内容:
 *          1.机器人状态的ros订阅和各状态的中转
 *          2.机器人运动控制的ros发布
 *          3.机器人参数的加载与管理
 *
 */
class Biped : public rclcpp::Node
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Biped(const std::string &name);

    struct LegConfig
    {
        double hip_offset_x;
        double hip_offset_y;
        double upper_leg_length;
        double lower_leg_length;
        double wheel_radius;
    };

    Vec3 legUpdateMsg(Vec &leg_q, Vec3 &leg_dq, Mat3 &J, int legID);

private:
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>>
        tf_sub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> rb_vel_sub_;
};

#endif