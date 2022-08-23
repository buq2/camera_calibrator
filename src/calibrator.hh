#pragma once

#include "types.hh"
#include <set>

namespace calibrator {

class Calibrator {
 public:
  Calibrator(const int img_width, const int img_height);

  void EstimateOpenCv(const std::vector<Points2D> &in_img_points,
                      const std::vector<Points3D> &in_world_points);

  void Estimate(const std::vector<Points2D> &in_img_points,
                const std::vector<Points3D> &in_world_points);

  void Optimize(const std::vector<Points2D> &in_img_points,
                const std::vector<Points3D> &in_world_points,
                std::vector<Quaternion> &qs, std::vector<Point3D> &ts);

  Matrix3 GetK() const { return K_; }
  DynamicVector GetDistortion() const { return dist_; }

   /// Set K
  /// \parma[in] K Calibration matrix
  void SetK(const Matrix3& K) {K_ = K;};

  /// Set distortion parameters
  /// \param[in] dist Distortion parameters
  void SetDistortion(const DynamicVector& dist) {dist_ = dist;};

  /// Force some distortion parameters to constants. These will not be optimized
  /// \param[in] distortion_idx Index of distortion parameter to be frozen/not optimized
  void ForceDistortionToConstant(const int distortion_idx);

  /// Undistort points
  /// \param[in] img_points Image points in pixels coordinates
  /// \return Image points in normalized coordinates
  Points2D Undistort(const Points2D &img_points);

  /// Distort points
  /// \param[in] normalized_points Normalized image points
  /// \return Image points in pixel coordinates
  Points2D Distort(const Points2D &normalized_points);
 private:
  int width_;
  int height_;
  Matrix3 K_{Matrix3::Identity()};
  DynamicVector dist_{DynamicVector::Zero(5)};
  std::set<int> constant_intrinsics_;
};  // class Calibrator

}  // namespace calibrator
