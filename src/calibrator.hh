#pragma once

#include "types.hh"

namespace calibrator {

class Calibrator {
 public:
  Calibrator(const int img_width, const int img_height);

  void Estimate(const std::vector<Points2D> &in_img_points,
                const std::vector<Points3D> &in_world_points);

  Matrix3 GetK() const { return K_; }
  DynamicVector GetDistortion() const { return dist_; }

 private:
  int width_;
  int height_;
  Matrix3 K_{Matrix3::Identity()};
  DynamicVector dist_{DynamicVector::Zero(5)};
};  // class Calibrator

}  // namespace calibrator
