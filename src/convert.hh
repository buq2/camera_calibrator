#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include "types.hh"

namespace calibrator {

template <typename T, typename T2>
cv::Matx<T, 3, 3> ToCvMat3x3(const Eigen::Matrix<T2, 3, 3>& in) {
  cv::Matx<T, 3, 3> out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out(i, j) = in(i, j);
    }
  }
  return out;
}

template <typename T, typename T2>
cv::Point3_<T> ToCvPoint3(const Eigen::Matrix<T2, 3, 1>& in) {
  cv::Point3_<T> out;
  out.x = in[0];
  out.y = in[1];
  out.z = in[2];
  return out;
}

template <typename T, typename T2>
cv::Point_<T> ToCvPoint2(const Eigen::Matrix<T2, 2, 1>& in) {
  cv::Point_<T> out;
  out.x = in[0];
  out.y = in[1];
  return out;
}

template <typename T, typename EigenMatrix>
cv::Mat ToCvMat(const EigenMatrix& in) {
  const auto rows = static_cast<int>(in.rows());
  const auto cols = static_cast<int>(in.cols());
  cv::Mat out(rows, cols, cv::DataType<T>::type);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      out.at<T>(i, j) = in(i, j);
    }
  }
  return out;
}

template <typename EigenMatrix, typename T>
EigenMatrix ToEigen(const cv::Mat& in) {
  const auto rows = in.rows;
  const auto cols = in.cols;
  EigenMatrix out(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      out(i, j) = static_cast<typename EigenMatrix::Scalar>(in.at<T>(i, j));
    }
  }
  return out;
}

template <typename EigenMatrix, typename T>
EigenMatrix ToEigen(const cv::Matx<T, 3, 3>& in) {
  EigenMatrix out(3, 3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out(i, j) = in(i, j);
    }
  }
  return out;
}

}  // namespace calibrator
