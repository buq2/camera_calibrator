#pragma once

#include "types.hh"

namespace calibrator {

class Calibrator {
 public:
  Calibrator(const int img_width, const int img_height);

  void Estimate(const std::vector<Points2D> &img_points,
                const std::vector<Points3D> &world_points);

 private:
  int width_;
  int height_;
  Matrix3 K_{Matrix3::Identity()};
  DynamicVector dist_{DynamicVector::Zero(5)};
};  // class Calibrator

}  // namespace calibrator
