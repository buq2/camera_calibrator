#include "calibrator.hh"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include "convert.hh"

using namespace calibrator;

Calibrator::Calibrator(const int img_width, const int img_height)
    : width_(img_width), height_(img_height) {}

void Calibrator::Estimate(const std::vector<Points2D> &in_img_points,
                          const std::vector<Points3D> &in_world_points) {
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> world_points;
  for (const auto &points : in_img_points) {
    image_points.emplace_back();
    auto &out_points = *image_points.rbegin();
    for (const auto &p : points) {
      out_points.push_back(ToCvPoint2<float>(p));
    }
  }
  for (const auto &points : in_world_points) {
    world_points.emplace_back();
    auto &out_points = *world_points.rbegin();
    for (const auto &p : points) {
      out_points.push_back(ToCvPoint3<float>(p));
    }
  }

  cv::Mat R, T;
  cv::Matx33f camera_matrix = ToCvMat3x3<float>(K_);
  cv::Mat dist_coeffs = ToCvMat<float>(dist_);
  // cv::calibrateCamera(world_points, image_points, cv::Size(width_, height_),
  // camera_matrix, dist_coeffs, rvecs, tvecs);
  cv::calibrateCamera(world_points, image_points, cv::Size(width_, height_),
                      camera_matrix, dist_coeffs, R, T);

  K_ = ToEigen<decltype(K_), float>(camera_matrix);

  // dist_coeffs is a double matrix
  dist_ = ToEigen<decltype(dist_), double>(dist_coeffs);
}
