#include "data_generator.hh"
#include <Eigen/QR>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include "convert.hh"

using namespace calibrator;

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

GeneratedData DataGenerator::GetDistortedPoints(const int num_p) {
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
  cv::Matx33f cameraMatrix = ToCvMat3x3<float>(K_);

  // Convert distortion coeffiecients to OpenCV format
  cv::Mat distCoeffs = ToCvMat<float>(dist_);

  // Generate points
  while (out.image.size() < num_p) {
    // Get random ideal image points u and v
    const auto u = rand_w_(gen_);
    const auto v = rand_h_(gen_);

    // Random distance from camera
    const auto z = rand_dist_(gen_);

    // Back project 2D point to 3D point which projects to the same location
    p[0] = u;
    p[1] = v;
    Eigen::Vector3f p_3d = K_inv * p;
    p_3d.normalize();
    p_3d = p_3d * z;

    // Generate distorted point using OpenCV
    std::vector<cv::Point3f> object;
    object.emplace_back(p_3d.x(), p_3d.y(), p_3d.z());
    std::vector<cv::Point2f> imagePoints(1);
    cv::projectPoints(object, rvec, tvec, cameraMatrix, distCoeffs,
                      imagePoints);

    const auto& imp = imagePoints[0];
    const auto u_dist = imp.x + rand_pixel_(gen_);
    const auto v_dist = imp.y + rand_pixel_(gen_);
    if (u_dist < 0.0f || u_dist >= width_ - 1 || v_dist < 0.0f ||
        v_dist >= height_ - 1) {
      // Distorted point outside of image, lets not use it
      std::cerr
          << "Generated point outside of the image. Ignoring generated point."
          << std::endl;
      continue;
    }
    out.world.push_back(p_3d);
    out.image.emplace_back(u_dist, v_dist);
  }

  return out;
}
