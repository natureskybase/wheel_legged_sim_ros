#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppType.h"
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

class LegController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegController(int ID);
    const int legID;
};

#endif