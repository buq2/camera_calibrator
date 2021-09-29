#include "data_generator.hh"
#include <Eigen/QR>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include "convert.hh"
#include "geometry.hh"

using namespace calibrator;

std::optional<Point2D> ProjectToCameraAndDistort(
    const cv::Matx33f& cameraMatrix, const cv::Mat& distCoeffs,
    const std::vector<cv::Point3f>& rvec, const std::vector<cv::Point3f>& tvec,
    const int width, const int height,
    std::uniform_real_distribution<float>& rand_pixel, std::mt19937& gen,
    const Point3D& p_3d) {
  // Generate distorted point using OpenCV
  std::vector<cv::Point3f> object;
  object.emplace_back(p_3d.x(), p_3d.y(), p_3d.z());
  std::vector<cv::Point2f> imagePoints(1);
  cv::projectPoints(object, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

  const auto& imp = imagePoints[0];
  const auto u_dist = imp.x + rand_pixel(gen);
  const auto v_dist = imp.y + rand_pixel(gen);
  if (u_dist < 0.0f || u_dist >= width - 1 || v_dist < 0.0f ||
      v_dist >= height - 1) {
    // Distorted point outside of image, lets not use it
    return std::nullopt;
  }

  return Point2D(u_dist, v_dist);
}

DataGenerator::DataGenerator(int img_width, int img_height)
    : width_(img_width),
      height_(img_height),
      rand_w_(0.0f, img_width - 1.0f),
      rand_h_(0.0f, img_height - 1.0f),
      rand_dist_(min_distace_, max_distance_),
      rand_pixel_(-noise_in_pixels_, noise_in_pixels_) {}

void DataGenerator::SetK(const Matrix3& K) { K_ = K; }

void DataGenerator::SetDistortion(const DynamicVector& dist) { dist_ = dist; }

void DataGenerator::SetNoiseInPixels(const float noise) {
  noise_in_pixels_ = noise;
  rand_pixel_ = std::uniform_real_distribution<float>(-noise_in_pixels_,
                                                      noise_in_pixels_);
}

Plane DataGenerator::GetRandomPlane(const Matrix3& K_inv) {
  Point3D p;
  p.x() = 0.0f;
  p.y() = 0.0f;
  p.z() = 1.0f;

  Eigen::Vector3f p1 = K_inv * p;
  p1.normalize();
  p1 *= rand_dist_(gen_);

  p.x() = static_cast<float>(width_ - 1);

  Eigen::Vector3f p2 = K_inv * p;
  p2.normalize();
  p2 *= rand_dist_(gen_);

  p.y() = static_cast<float>(height_ - 1);

  Eigen::Vector3f p3 = K_inv * p;
  p3.normalize();
  p3 *= rand_dist_(gen_);

  return EstimatePlaneFinite(p1, p2, p3);
}

GeneratedData DataGenerator::GetDistortedPointsPlanar(const int num_p) {
  GeneratedData out;

  // Invert K for back projection
  const auto K_inv = K_.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Vector3f p;
  p[2] = 1.0;

  // For OpenCV use
  // Create rotation and translation vectors which are just zeros
  std::vector<cv::Point3f> rvec;
  rvec.emplace_back(0.0f, 0.0f, 0.0f);
  std::vector<cv::Point3f> tvec;
  tvec.emplace_back(0.0f, 0.0f, 0.0f);

  // Convert cameramatrix to format used by OpenCV
  cv::Matx33f camera_matrix = ToCvMat3x3<float>(K_);

  // Convert distortion coeffiecients to OpenCV format
  cv::Mat dist_coeffs = ToCvMat<float>(dist_);

  Plane plane = GetRandomPlane(K_inv);
  const auto R = RotationMatrixFromPlane(plane);

  // Generate points
  while (out.image.size() < num_p) {
    const auto p_3d = GetRandom3DPointVisibleToCamera(K_inv);
    // Project the 3D point to the plane using direction of the point
    // as projection direction
    const auto p_3d_planar = ProjectToPlane(plane, p_3d, p_3d);
    const auto p_3d_rotated = R * p_3d_planar;

    const auto p_2d_maybe = ProjectToCameraAndDistort(
        camera_matrix, dist_coeffs, rvec, tvec, width_, height_, rand_pixel_,
        gen_, p_3d_planar);
    if (!p_2d_maybe) {
      std::cerr
          << "Generated point outside of the image. Ignoring generated point."
          << std::endl;
      continue;
    }

    out.world.emplace_back(p_3d_rotated(0), p_3d_rotated(1), 0.0f);
    out.image.push_back(*p_2d_maybe);
  }

  return out;
}

Point3D DataGenerator::GetRandomPixel() {
  Point3D p;
  p[0] = rand_w_(gen_);
  p[1] = rand_h_(gen_);
  p[2] = 1.0f;
  return p;
}

Point3D DataGenerator::GetRandom3DPointVisibleToCamera(const Matrix3& K_inv) {
  // Get random ideal image points u and v
  const auto p = GetRandomPixel();

  // Back project 2D point to 3D point which projects to the same location
  Eigen::Vector3f p_3d = K_inv * p;
  p_3d.normalize();
  // Random distance from camera
  const auto z = rand_dist_(gen_);
  p_3d = p_3d * z;

  return p_3d;
}

GeneratedData DataGenerator::GetDistortedPoints(const int num_p) {
  GeneratedData out;

  // Invert K for back projection
  const auto K_inv = K_.completeOrthogonalDecomposition().pseudoInverse();

  // For OpenCV use
  // Create rotation and translation vectors which are just zeros
  std::vector<cv::Point3f> rvec;
  rvec.emplace_back(0.0f, 0.0f, 0.0f);
  std::vector<cv::Point3f> tvec;
  tvec.emplace_back(0.0f, 0.0f, 0.0f);

  // Convert cameramatrix to format used by OpenCV
  cv::Matx33f camera_matrix = ToCvMat3x3<float>(K_);

  // Convert distortion coeffiecients to OpenCV format
  cv::Mat dist_coeffs = ToCvMat<float>(dist_);

  // Generate points
  while (out.image.size() < num_p) {
    const auto p_3d = GetRandom3DPointVisibleToCamera(K_inv);
    const auto p_2d_maybe =
        ProjectToCameraAndDistort(camera_matrix, dist_coeffs, rvec, tvec,
                                  width_, height_, rand_pixel_, gen_, p_3d);
    if (!p_2d_maybe) {
      std::cerr
          << "Generated point outside of the image. Ignoring generated point."
          << std::endl;
      continue;
    }
    out.world.push_back(p_3d);
    out.image.push_back(*p_2d_maybe);
  }

  return out;
}
