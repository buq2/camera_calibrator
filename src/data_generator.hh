#pragma once

#include <optional>
#include <random>
#include "types.hh"

namespace calibrator {

struct GeneratedData {
  Points2D image;
  Points3D world;
};

class DataGenerator {
 public:
  DataGenerator(int img_width, int img_height);
  void SetK(const Matrix3& K);
  Matrix3 GetK() const { return K_; }
  int GetWidth() const { return width_; }
  int GetHeight() const { return height_; }
  DynamicVector GetDistortion() const { return dist_; }
  void SetDistortion(const DynamicVector& dist);
  void SetNoiseInPixels(const float noise);
  GeneratedData GetDistortedPoints(const int num_p = 100);
  GeneratedData GetDistortedPointsPlanar(const int num_p = 100);

 private:
  Point3D GetRandomPixel();
  Point3D GetRandom3DPointVisibleToCamera(const Matrix3& K_inv);

  // Create three points which are at the corners of the
  // camera frustrum. This will define a plane.
  Plane GetRandomPlane(const Matrix3& K_inv);

 private:
  int width_{0};
  int height_{0};
  float min_distace_{0.2f};
  float max_distance_{1.0f};
  float noise_in_pixels_{0.0f};
  Matrix3 K_{Matrix3::Identity()};
  DynamicVector dist_{};
  std::mt19937 gen_{0};
  std::uniform_real_distribution<float> rand_w_;
  std::uniform_real_distribution<float> rand_h_;
  std::uniform_real_distribution<float> rand_dist_;
  std::uniform_real_distribution<float> rand_pixel_;
};

}  // namespace calibrator
