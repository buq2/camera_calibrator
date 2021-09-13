#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace calibrator {

using Point2D = Eigen::Vector2f;
using Point3D = Eigen::Vector3f;
using Points2D = std::vector<Point2D, Eigen::aligned_allocator<Point2D> >;
using Points3D = std::vector<Point3D, Eigen::aligned_allocator<Point3D> >;
using Matrix3 = Eigen::Matrix3f;
using DynamicVector = Eigen::VectorXf;
using Plane = Eigen::Vector4f;

}  // namespace calibrator
