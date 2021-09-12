#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace calibrator {

using Points2D =
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >;
using Points3D =
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;
using Matrix3 = Eigen::Matrix3f;
using DynamicVector = Eigen::VectorXf;

}  // namespace calibrator
