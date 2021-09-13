#include "calibrator.hh"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include "convert.hh"
#include "geometry.hh"

using namespace calibrator;

Calibrator::Calibrator(const int img_width, const int img_height)
    : width_(img_width), height_(img_height) {}

void Calibrator::EstimateOpenCv(const std::vector<Points2D> &in_img_points,
                                const std::vector<Points3D> &in_world_points) {
  assert(in_img_points.size() == in_world_points.size());
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

void Calibrator::Estimate(const std::vector<Points2D> &in_img_points,
                          const std::vector<Points3D> &in_world_points) {
  assert(in_img_points.size() == in_world_points.size());
  const auto num_imgs = in_img_points.size();

  std::vector<Matrix3> Hs;
  for (size_t i = 0; i < num_imgs; ++i) {
    Hs.push_back(EstimateHomography(in_world_points[i], in_img_points[i]));
  }
  K_ = EstimateKFromHomographies(Hs);

  std::vector<Quaternion> qs;
  std::vector<Point3D> ts;
  const auto K_inv = K_.inverse();
  for (const auto &H : Hs) {
    const auto &[R, t] = RecoverExtrinsics(K_inv, H);
    qs.emplace_back(R);
    ts.push_back(t);
  }
}
