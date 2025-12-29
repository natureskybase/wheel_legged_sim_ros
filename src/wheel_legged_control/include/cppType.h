#ifndef CPP_TYPE_H
#define CPP_TYPE_H

#include <eigen3/Eigen/Dense>

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Mat3 = Eigen::Matrix<double, 3, 3>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using DMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

// Mat3 crossMatrix(const Vec3 &v);
// Mat3 quaternionToRotationMatrix(Vec4 &q);

#endif
